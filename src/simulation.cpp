#include "swarmgrid/simulation.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <ranges>
#include <thread>
#include <mutex>
#include <future>
#include <execution>

namespace swarmgrid {

Simulation::Simulation(SimulationConfig config,
                      std::unique_ptr<ports::IMapLoader> map_loader,
                      std::unique_ptr<ports::INetwork> network)
    : config_(std::move(config))
    , map_loader_(std::move(map_loader))
    , network_(std::move(network)) {
}

Simulation::Simulation(SimulationConfig config,
                      std::unique_ptr<ports::INetwork> network)
    : config_(std::move(config))
    , network_(std::move(network)) {
}

bool Simulation::initialize() {
    spdlog::info("Initializing simulation with seed {}", config_.seed);

    if (!config_.world.has_value() && map_loader_) {
        // Load from file if world not provided directly
        auto world = map_loader_->load(config_.map_path, config_.num_agents, config_.seed);
        if (!world) {
            spdlog::error("Failed to load world from map");
            return false;
        }
        config_.world = std::move(*world);
    }

    if (!config_.world.has_value()) {
        spdlog::error("No world provided");
        return false;
    }

    world_manager_ = core::WorldManager(std::move(*config_.world));
    planner_ = std::make_unique<core::PathPlanner>(world_manager_->get_world());

    // Initialize agent controllers
    agent_controllers_.clear();
    {
        std::lock_guard<std::mutex> lock(world_access_mutex_);
        for (const auto& agent : world_manager_->get_world().agents) {
            AgentController controller;
            controller.id = agent.id;
            controller.last_intent = agent.pos;
            agent_controllers_.push_back(controller);
        }
    }

    spdlog::info("Initialized {} agents", agent_controllers_.size());
    initialized_ = true;
    return true;
}

bool Simulation::run() {
    if (!world_manager_) {
        spdlog::error("Simulation not initialized");
        return false;
    }

    spdlog::info("Starting simulation");
    metrics_collector_.reset();
    metrics_collector_.start_timer();

    while (!check_termination()) {
        step_internal();
        world_manager_->advance_tick();

        if (world_manager_->get_world().current_tick >= config_.max_ticks) {
            spdlog::warn("Reached maximum steps limit");
            break;
        }
    }

    metrics_collector_.stop_timer();
    metrics_collector_.set_makespan(world_manager_->get_world().current_tick);

    // Check for collisions
    auto colliding_agents = world_manager_->detect_collisions();
    if (!colliding_agents.empty()) {
        spdlog::error("Collision detected! {} agents involved", colliding_agents.size());
        metrics_collector_.record_collision();
    }

    save_outputs();

    spdlog::info("Simulation completed in {} ticks", world_manager_->get_world().current_tick);
    return true;
}

void Simulation::step_internal() {
    if (config_.verbose) {
        log_tick_state();
    }

    // Fixed timing: receive fresh data BEFORE planning
    receive_and_update_local_state();
    plan_agent_moves();
    broadcast_intents();
    validate_pre_execution_conflicts();
    detect_and_resolve_deadlocks();
    execute_moves();
    detect_and_handle_collisions();

    // Record trace
    core::TickTrace trace;
    trace.tick = world_manager_->get_world().current_tick;
    trace.active_agents = world_manager_->count_active_agents();
    trace.messages_sent = 0;  // Will be updated in exchange_intents

    {
        std::lock_guard<std::mutex> lock(world_access_mutex_);
        for (const auto& agent : world_manager_->get_world().agents) {
            trace.agent_positions.emplace_back(agent.id, agent.pos);
        }
    }

    metrics_collector_.record_tick_trace(trace);
}

void Simulation::plan_agent_moves() {
    // Get number of hardware threads available
    const unsigned int num_threads = std::thread::hardware_concurrency();
    if (config_.verbose) {
        static bool logged_threads = false;
        if (!logged_threads) {
            spdlog::debug("Using {} threads for parallel agent planning", num_threads);
            logged_threads = true;
        }
    }

    // Parallel planning using async tasks
    std::vector<std::future<void>> futures;

    for (auto& controller : agent_controllers_) {
        futures.push_back(std::async(std::launch::async, [this, &controller]() {
            // Thread-safe access to world agents
            core::AgentState agent_state;
            bool agent_found = false;
            core::Tick current_tick;

            {
                std::lock_guard<std::mutex> lock(world_access_mutex_);
                auto& world = world_manager_->get_world();
                current_tick = world.current_tick;
                auto agent_it = std::ranges::find_if(world.agents,
                    [&](const auto& a) { return a.id == controller.id; });

                if (agent_it != world.agents.end()) {
                    agent_state = *agent_it;  // Make a copy for thread safety
                    agent_found = true;
                }
            }

            if (!agent_found || agent_state.at_goal || agent_state.collision_stopped) {
                return;
            }

            if (controller.needs_replan || controller.current_path.empty() ||
                controller.path_index >= controller.current_path.size()) {

            // Clear old reservations from LOCAL table
            planner_->clear_reservations(controller.id, controller.local_reservations);

            // Plan using LOCAL reservation table based on received messages
            auto path = planner_->plan_path(
                agent_state.pos,
                agent_state.goal,
                controller.local_reservations,  // Use local view!
                controller.id,
                current_tick
            );

            if (!path.empty()) {
                controller.current_path = path;
                controller.path_index = 0;
                controller.needs_replan = false;
                controller.wait_counter = 0;

                // Commit to LOCAL reservations
                planner_->commit_reservations(path, controller.id, controller.local_reservations, current_tick);

                if (config_.verbose) {
                    spdlog::debug("Agent {} planned path of length {}",
                                boost::uuids::to_string(controller.id), path.size());
                }
            } else {
                // No path found, wait and try again
                controller.wait_counter++;
                if (controller.wait_counter >= AgentController::MAX_WAIT) {
                    controller.needs_replan = true;
                        metrics_collector_.record_replan();
                    }
                }
            }
        }));
    }

    // Wait for all planning tasks to complete
    for (auto& future : futures) {
        future.wait();
    }
}

void Simulation::receive_and_update_local_state() {
    auto current_tick = world_manager_->get_world().current_tick;

    // Receive intents from other agents and build local view
    for (auto& controller : agent_controllers_) {
        // Clear local reservations before rebuilding from messages
        controller.local_reservations.clear();

        auto messages = network_->receive(controller.id, current_tick);

        for (const auto& msg : messages) {
            // Update vector clock upon receiving any message (Lamport timestamps)
            for (const auto& [agent_id, clock_value] : msg.vector_clock) {
                controller.vector_clock[agent_id] = std::max(controller.vector_clock[agent_id], clock_value);
            }
            controller.local_clock = std::max(controller.local_clock, controller.vector_clock[controller.id]) + 1;
            controller.vector_clock[controller.id] = controller.local_clock;

            // Handle different message types
            if (msg.type == ports::MessageType::PATH_ANNOUNCEMENT || msg.type == ports::MessageType::GOAL_REACHED) {
                // Store the other agent's intention
                AgentController::OtherAgentIntent intent{
                    msg.from,
                    msg.next,
                    msg.timestamp,
                    msg.planned_path
                };

                // Update or add this agent's intention
                auto it = std::find_if(controller.known_intents.begin(), controller.known_intents.end(),
                    [&msg](const auto& i) { return i.agent_id == msg.from; });

                if (it != controller.known_intents.end()) {
                    *it = intent;
                } else {
                    controller.known_intents.push_back(intent);
                }

                // Add their planned path to our local reservation table
                if (!msg.planned_path.empty()) {
                    planner_->commit_reservations(msg.planned_path, msg.from,
                                                 controller.local_reservations, current_tick);
                }

                // Check for conflicts with our current plan (look ahead several steps)
                if (!controller.current_path.empty() && !msg.planned_path.empty()) {
                    // Check next 15 positions for potential conflicts (was 3, causing undetected collisions)
                    const int LOOKAHEAD_STEPS = 15;

                    for (int our_offset = 0; our_offset < LOOKAHEAD_STEPS &&
                         controller.path_index + our_offset < controller.current_path.size(); ++our_offset) {

                        for (int their_offset = 0; their_offset < LOOKAHEAD_STEPS &&
                             their_offset < msg.planned_path.size(); ++their_offset) {

                            // Check if we'll be at the same place at the same time
                            if (our_offset == their_offset &&
                                controller.current_path[controller.path_index + our_offset] ==
                                msg.planned_path[their_offset]) {

                                // Conflict detected, use vector clock or agent ID to decide who replans
                                bool should_replan = false;

                                // First check if we can use vector clock ordering
                                auto their_clock_it = msg.vector_clock.find(msg.from);
                                auto our_clock_it = controller.vector_clock.find(controller.id);

                                if (their_clock_it != msg.vector_clock.end() && our_clock_it != controller.vector_clock.end()) {
                                    // Use vector clock - agent with newer (larger) clock value wins
                                    if (their_clock_it->second > our_clock_it->second) {
                                        should_replan = true;
                                    }
                                } else {
                                    // Fallback to agent ID comparison
                                    should_replan = (msg.from < controller.id);
                                }

                                if (should_replan) {
                                    controller.needs_replan = true;
                                    metrics_collector_.record_replan();
                                    if (config_.verbose) {
                                        spdlog::debug("Agent {} detected future conflict at step {}, will replan",
                                                    boost::uuids::to_string(controller.id), our_offset);
                                    }
                                    break;
                                }
                            }
                        }

                        if (controller.needs_replan) break;
                    }
                }
            } else if (msg.type == ports::MessageType::STATE_SYNC) {
                // Handle full state synchronization with vector clock ordering
                if (msg.full_state != nullptr) {
                    // Use vector clocks to determine if this state update is causally newer
                    bool should_accept_state = false;
                    auto& last_seq = controller.last_seen_sequence[msg.from];

                    // Simple check: if sequence number is higher, accept it
                    // In a full vector clock implementation, we'd check if the received clock
                    // "happens-after" our current clock
                    if (msg.sequence_number > last_seq) {
                        should_accept_state = true;
                    }

                    if (should_accept_state) {
                        // Reconcile with received state - merge reservations
                        for (const auto& reservation : *msg.full_state) {
                            // Check if we already have this reservation
                            auto existing = controller.local_reservations.find(reservation.key);

                            if (existing == controller.local_reservations.end()) {
                                // Add new reservation we didn't know about
                                controller.local_reservations.insert(reservation);
                            } else {
                                // We have conflicting reservations - use vector clock or agent ID as tie-breaker
                                bool should_replace = false;

                                // Check if we can use vector clock ordering
                                auto their_clock_it = msg.vector_clock.find(reservation.agent_id);
                                auto existing_clock_it = controller.vector_clock.find(existing->agent_id);

                                if (their_clock_it != msg.vector_clock.end() && existing_clock_it != controller.vector_clock.end()) {
                                    // Use vector clock - reservation from agent with newer clock wins
                                    should_replace = (their_clock_it->second > existing_clock_it->second);
                                } else {
                                    // Fallback to agent ID comparison
                                    should_replace = (reservation.agent_id < existing->agent_id);
                                }

                                if (should_replace) {
                                    // Replace with the reservation from "higher priority" agent
                                    controller.local_reservations.erase(existing);
                                    controller.local_reservations.insert(reservation);
                                }
                            }
                        }

                        last_seq = msg.sequence_number;
                        controller.last_state_received = current_tick;

                        if (config_.verbose) {
                            spdlog::debug("Agent {} reconciled state from agent {} (seq: {}, vector clock size: {})",
                                        boost::uuids::to_string(controller.id),
                                        boost::uuids::to_string(msg.from),
                                        msg.sequence_number,
                                        msg.vector_clock.size());
                        }
                    }
                }
            }
        }
    }
}

void Simulation::broadcast_intents() {
    // Clear old intentions from previous ticks (time-based cleanup)
    auto current_tick = world_manager_->get_world().current_tick;
    for (auto& controller : agent_controllers_) {
        // Remove stale intentions older than a few ticks
        controller.known_intents.erase(
            std::remove_if(controller.known_intents.begin(), controller.known_intents.end(),
                [current_tick](const auto& intent) {
                    return intent.timestamp + 5 < current_tick;  // Keep intentions for 5 ticks
                }),
            controller.known_intents.end());
    }

    // Track network stats before sending
    auto stats_before = network_->get_stats();

    // Mesh-style broadcasting: Send both path announcements AND full state periodically
    constexpr int REDUNDANCY_FACTOR = 3;

    for (auto& controller : agent_controllers_) {
        // Find the corresponding agent state to check if at goal
        core::AgentState agent_state;
        bool agent_found = false;

        {
            std::lock_guard<std::mutex> lock(world_access_mutex_);
            auto& world = world_manager_->get_world();
            auto agent_it = std::ranges::find_if(world.agents,
                [&](auto& a) { return a.id == controller.id; });

            if (agent_it != world.agents.end()) {
                agent_state = *agent_it;  // Make a copy for thread safety
                agent_found = true;
            }
        }

        // Always send path announcements, even for agents at goal
        ports::Message path_msg;
        path_msg.from = controller.id;
        path_msg.type = ports::MessageType::PATH_ANNOUNCEMENT;

        if (agent_found) {
            // Use current position as "next" for agents at goal or collision-stopped
            if (agent_state.at_goal || agent_state.collision_stopped || controller.current_path.empty()) {
                // For agents at goal, use high-priority GOAL_REACHED message type
                if (agent_state.at_goal) {
                    path_msg.type = ports::MessageType::GOAL_REACHED;
                }

                path_msg.next = agent_state.pos;  // Stay at current position

                // For goal-reached or collision-stopped agents, broadcast a long-term reservation
                // to ensure other agents know this square is permanently occupied
                std::vector<core::Cell> permanent_path;
                const int PERMANENT_RESERVATION_LENGTH = 200;  // Increased reservation length
                for (int i = 0; i < PERMANENT_RESERVATION_LENGTH; ++i) {
                    permanent_path.push_back(agent_state.pos);
                }
                path_msg.planned_path = permanent_path;
            } else {
                path_msg.next = controller.path_index < controller.current_path.size() ?
                               controller.current_path[controller.path_index] : controller.last_intent;

                // FIX: Broadcast only the REMAINING path from current position forward
                if (controller.path_index < controller.current_path.size()) {
                    path_msg.planned_path = std::vector<core::Cell>(
                        controller.current_path.begin() + controller.path_index,
                        controller.current_path.end()
                    );
                } else {
                    path_msg.planned_path = {}; // No path left
                }
            }

            // Set message metadata
            path_msg.timestamp = current_tick;
            controller.local_clock++;
            controller.vector_clock[controller.id] = controller.local_clock;
            path_msg.vector_clock = controller.vector_clock;
        }

        // Send with redundancy
        for (int i = 0; i < REDUNDANCY_FACTOR; ++i) {
            network_->send(path_msg);
        }

        // Mesh periodic state sync: Send full state occasionally
        if (current_tick % AgentController::STATE_BROADCAST_INTERVAL == 0 ||
            current_tick - controller.last_state_received > AgentController::STALE_STATE_THRESHOLD) {

            ports::Message state_msg;
            state_msg.from = controller.id;
            state_msg.type = ports::MessageType::STATE_SYNC;
            state_msg.timestamp = current_tick;
            state_msg.sequence_number = current_tick;
            controller.local_clock++;
            controller.vector_clock[controller.id] = controller.local_clock;
            state_msg.vector_clock = controller.vector_clock;

            // Attach full reservation table
            state_msg.full_state = std::make_shared<core::ReservationTable>(controller.local_reservations);

            controller.last_state_broadcast = current_tick;

            // Send with redundancy
            for (int i = 0; i < REDUNDANCY_FACTOR; ++i) {
                network_->send(state_msg);
            }
        }
    }

    // Get network stats after sending to track actual drops
    auto stats_after = network_->get_stats();
    uint64_t messages_sent_this_tick = stats_after.sent - stats_before.sent;
    uint64_t messages_dropped_this_tick = stats_after.dropped - stats_before.dropped;

    // Record the actual statistics
    for (uint64_t i = 0; i < messages_sent_this_tick; ++i) {
        metrics_collector_.record_message_sent();
    }
    for (uint64_t i = 0; i < messages_dropped_this_tick; ++i) {
        metrics_collector_.record_message_dropped();
    }
}

void Simulation::exchange_intents() {
    // Clear old intentions from previous ticks (time-based cleanup)
    auto current_tick = world_manager_->get_world().current_tick;
    for (auto& controller : agent_controllers_) {
        // Remove stale intentions older than a few ticks
        controller.known_intents.erase(
            std::remove_if(controller.known_intents.begin(), controller.known_intents.end(),
                [current_tick](const auto& intent) {
                    return intent.timestamp + 5 < current_tick;  // Keep intentions for 5 ticks
                }),
            controller.known_intents.end());
    }

    // Track network stats before sending
    auto stats_before = network_->get_stats();

    // Mesh-style broadcasting: Send both path announcements AND full state periodically
    constexpr int REDUNDANCY_FACTOR = 3;

    for (auto& controller : agent_controllers_) {
        // Find the corresponding agent state to check if at goal
        core::AgentState agent_state;
        bool agent_found = false;
        {
            std::lock_guard<std::mutex> lock(world_access_mutex_);
            auto& world = world_manager_->get_world();
            auto agent_it = std::ranges::find_if(world.agents,
                [&](const auto& a) { return a.id == controller.id; });
            if (agent_it != world.agents.end()) {
                agent_state = *agent_it;
                agent_found = true;
            }
        }

        // Always send path announcements, even for agents at goal
        ports::Message path_msg;
        path_msg.from = controller.id;
        path_msg.type = ports::MessageType::PATH_ANNOUNCEMENT;

        if (agent_found) {
            // Use current position as "next" for agents at goal or collision-stopped
            if (agent_state.at_goal || agent_state.collision_stopped || controller.current_path.empty()) {
                // For agents at goal, use high-priority GOAL_REACHED message type
                if (agent_state.at_goal) {
                    path_msg.type = ports::MessageType::GOAL_REACHED;
                }

                path_msg.next = agent_state.pos;  // Stay at current position

                // For goal-reached or collision-stopped agents, broadcast a long-term reservation
                // to ensure other agents know this square is permanently occupied
                std::vector<core::Cell> permanent_path;
                const int PERMANENT_RESERVATION_LENGTH = 200;  // Increased reservation length
                for (int i = 0; i < PERMANENT_RESERVATION_LENGTH; ++i) {
                    permanent_path.push_back(agent_state.pos);
                }
                path_msg.planned_path = permanent_path;
            } else {
                path_msg.next = controller.path_index < controller.current_path.size() ?
                               controller.current_path[controller.path_index] : controller.last_intent;

                // FIX: Broadcast only the REMAINING path from current position forward
                if (controller.path_index < controller.current_path.size()) {
                    path_msg.planned_path = std::vector<core::Cell>(
                        controller.current_path.begin() + controller.path_index,
                        controller.current_path.end()
                    );
                } else {
                    path_msg.planned_path = {}; // No path left
                }
            }
        } else {
            // Fallback for missing agent
            path_msg.next = controller.last_intent;

            // FIX: Even in fallback, broadcast only remaining path
            if (controller.path_index < controller.current_path.size()) {
                path_msg.planned_path = std::vector<core::Cell>(
                    controller.current_path.begin() + controller.path_index,
                    controller.current_path.end()
                );
            } else {
                path_msg.planned_path = {}; // No path left
            }
        }

        path_msg.timestamp = current_tick;
        path_msg.sequence_number = current_tick;  // Simple sequence numbering

        // Update vector clock before sending
        controller.local_clock++;
        controller.vector_clock[controller.id] = controller.local_clock;
        path_msg.vector_clock = controller.vector_clock;

        // Send multiple times for redundancy
        for (int i = 0; i < REDUNDANCY_FACTOR; ++i) {
            network_->send(path_msg);
        }

        // Periodically broadcast FULL STATE for mesh synchronization
        bool should_broadcast_state =
            (current_tick - controller.last_state_broadcast >= AgentController::STATE_BROADCAST_INTERVAL) ||
            (current_tick - controller.last_state_received >= AgentController::STALE_STATE_THRESHOLD);

        if (should_broadcast_state) {
            ports::Message state_msg;
            state_msg.from = controller.id;
            state_msg.type = ports::MessageType::STATE_SYNC;
            state_msg.timestamp = current_tick;
            state_msg.sequence_number = current_tick;

            // Create a copy of our local reservation table to share
            state_msg.full_state = std::make_shared<core::ReservationTable>(controller.local_reservations);

            // Update vector clock before sending
            controller.local_clock++;
            controller.vector_clock[controller.id] = controller.local_clock;
            state_msg.vector_clock = controller.vector_clock;

            controller.last_state_broadcast = current_tick;

            // Send state sync message (with redundancy)
            for (int i = 0; i < REDUNDANCY_FACTOR; ++i) {
                network_->send(state_msg);
            }

            if (config_.verbose) {
                spdlog::debug("Agent {} broadcasting full state at tick {}",
                            boost::uuids::to_string(controller.id), current_tick);
            }
        }
    }

    // Get network stats after sending to track actual drops
    auto stats_after = network_->get_stats();
    uint64_t messages_sent_this_tick = stats_after.sent - stats_before.sent;
    uint64_t messages_dropped_this_tick = stats_after.dropped - stats_before.dropped;

    // Record the actual statistics
    for (uint64_t i = 0; i < messages_sent_this_tick; ++i) {
        metrics_collector_.record_message_sent();
    }
    for (uint64_t i = 0; i < messages_dropped_this_tick; ++i) {
        metrics_collector_.record_message_dropped();
    }

    // Receive intents from other agents and build local view
    for (auto& controller : agent_controllers_) {
        // Clear local reservations before rebuilding from messages
        controller.local_reservations.clear();

        auto messages = network_->receive(controller.id, current_tick);

        for (const auto& msg : messages) {
            // Update vector clock upon receiving any message (Lamport timestamps)
            for (const auto& [agent_id, clock_value] : msg.vector_clock) {
                controller.vector_clock[agent_id] = std::max(controller.vector_clock[agent_id], clock_value);
            }
            controller.local_clock = std::max(controller.local_clock, controller.vector_clock[controller.id]) + 1;
            controller.vector_clock[controller.id] = controller.local_clock;

            // Handle different message types
            if (msg.type == ports::MessageType::PATH_ANNOUNCEMENT || msg.type == ports::MessageType::GOAL_REACHED) {
                // Store the other agent's intention
                AgentController::OtherAgentIntent intent{
                    msg.from,
                    msg.next,
                    msg.timestamp,
                    msg.planned_path
                };

                // Update or add this agent's intention
                auto it = std::find_if(controller.known_intents.begin(), controller.known_intents.end(),
                    [&msg](const auto& i) { return i.agent_id == msg.from; });

                if (it != controller.known_intents.end()) {
                    *it = intent;
                } else {
                    controller.known_intents.push_back(intent);
                }

                // Add their planned path to our local reservation table
                if (!msg.planned_path.empty()) {
                    planner_->commit_reservations(msg.planned_path, msg.from,
                                                 controller.local_reservations, current_tick);
                }

                // Check for conflicts with our current plan (look ahead several steps)
                if (!controller.current_path.empty() && !msg.planned_path.empty()) {
                    // Check next 15 positions for potential conflicts (was 3, causing undetected collisions)
                    const int LOOKAHEAD_STEPS = 15;

                    for (int our_offset = 0; our_offset < LOOKAHEAD_STEPS &&
                         controller.path_index + our_offset < controller.current_path.size(); ++our_offset) {

                        for (int their_offset = 0; their_offset < LOOKAHEAD_STEPS &&
                             their_offset < msg.planned_path.size(); ++their_offset) {

                            // Check if we'll be at the same place at the same time
                            if (our_offset == their_offset &&
                                controller.current_path[controller.path_index + our_offset] ==
                                msg.planned_path[their_offset]) {

                                // Conflict detected, use vector clock or agent ID to decide who replans
                                bool should_replan = false;

                                // First check if we can use vector clock ordering
                                auto their_clock_it = msg.vector_clock.find(msg.from);
                                auto our_clock_it = controller.vector_clock.find(controller.id);

                                if (their_clock_it != msg.vector_clock.end() && our_clock_it != controller.vector_clock.end()) {
                                    // Use vector clock - agent with newer (larger) clock value wins
                                    if (their_clock_it->second > our_clock_it->second) {
                                        should_replan = true;
                                    }
                                } else {
                                    // Fallback to agent ID comparison
                                    should_replan = (msg.from < controller.id);
                                }

                                if (should_replan) {
                                    controller.needs_replan = true;
                                    metrics_collector_.record_replan();
                                    if (config_.verbose) {
                                        spdlog::debug("Agent {} detected future conflict at step {}, will replan",
                                                    boost::uuids::to_string(controller.id), our_offset);
                                    }
                                    break;
                                }
                            }
                        }

                        if (controller.needs_replan) break;
                    }
                }
            } else if (msg.type == ports::MessageType::STATE_SYNC) {
                // Handle full state synchronization with vector clock ordering
                if (msg.full_state != nullptr) {
                    // Use vector clocks to determine if this state update is causally newer
                    bool should_accept_state = false;
                    auto& last_seq = controller.last_seen_sequence[msg.from];

                    // Simple check: if sequence number is higher, accept it
                    // In a full vector clock implementation, we'd check if the received clock
                    // "happens-after" our current clock
                    if (msg.sequence_number > last_seq) {
                        should_accept_state = true;
                    }

                    if (should_accept_state) {
                        // Reconcile with received state - merge reservations
                        for (const auto& reservation : *msg.full_state) {
                            // Check if we already have this reservation
                            auto existing = controller.local_reservations.find(reservation.key);

                            if (existing == controller.local_reservations.end()) {
                                // Add new reservation we didn't know about
                                controller.local_reservations.insert(reservation);
                            } else {
                                // We have conflicting reservations - use vector clock or agent ID as tie-breaker
                                bool should_replace = false;

                                // Check if we can use vector clock ordering
                                auto their_clock_it = msg.vector_clock.find(reservation.agent_id);
                                auto existing_clock_it = controller.vector_clock.find(existing->agent_id);

                                if (their_clock_it != msg.vector_clock.end() && existing_clock_it != controller.vector_clock.end()) {
                                    // Use vector clock - reservation from agent with newer clock wins
                                    should_replace = (their_clock_it->second > existing_clock_it->second);
                                } else {
                                    // Fallback to agent ID comparison
                                    should_replace = (reservation.agent_id < existing->agent_id);
                                }

                                if (should_replace) {
                                    // Replace with the reservation from "higher priority" agent
                                    controller.local_reservations.erase(existing);
                                    controller.local_reservations.insert(reservation);
                                }
                            }
                        }

                        last_seq = msg.sequence_number;
                        controller.last_state_received = current_tick;

                        if (config_.verbose) {
                            spdlog::debug("Agent {} reconciled state from agent {} (seq: {}, vector clock size: {})",
                                        boost::uuids::to_string(controller.id),
                                        boost::uuids::to_string(msg.from),
                                        msg.sequence_number,
                                        msg.vector_clock.size());
                        }
                    }
                }
            }
        }
    }
}

void Simulation::validate_pre_execution_conflicts() {
    auto& world = world_manager_->get_world();
    std::unordered_map<core::Cell, std::vector<boost::uuids::uuid>, core::CellHash> next_positions;

    // Collect all intended next moves
    for (auto& controller : agent_controllers_) {
        // Thread-safe access to world agents
        bool should_skip = false;
        {
            std::lock_guard<std::mutex> lock(world_access_mutex_);
            auto agent_it = std::ranges::find_if(world.agents,
                [&](auto& a) { return a.id == controller.id; });

            if (agent_it == world.agents.end() || agent_it->at_goal || agent_it->collision_stopped) {
                should_skip = true;
            }
        }

        if (should_skip) {
            continue;
        }

        if (controller.path_index < controller.current_path.size()) {
            auto next_pos = controller.current_path[controller.path_index];
            next_positions[next_pos].push_back(controller.id);
        }
    }

    // Check for conflicts and force replanning
    for (const auto& [pos, agent_ids] : next_positions) {
        if (agent_ids.size() > 1) {
            // Multiple agents want the same position - force all to replan
            for (const auto& agent_id : agent_ids) {
                auto controller_it = std::ranges::find_if(agent_controllers_,
                    [&](auto& c) { return c.id == agent_id; });

                if (controller_it != agent_controllers_.end()) {
                    controller_it->needs_replan = true;
                    metrics_collector_.record_replan();
                    if (config_.verbose) {
                        spdlog::debug("Agent {} pre-execution conflict detected at ({},{}), forcing replan",
                                    boost::uuids::to_string(agent_id), pos.x, pos.y);
                    }
                }
            }
        }
    }

    // If conflicts detected, do emergency replanning
    bool conflicts_found = false;
    for (auto& controller : agent_controllers_) {
        if (controller.needs_replan) {
            conflicts_found = true;
            break;
        }
    }

    if (conflicts_found) {
        if (config_.verbose) {
            spdlog::warn("Pre-execution conflicts detected, performing emergency replanning");
        }

        // Quick emergency replanning (non-parallel for simplicity)
        for (auto& controller : agent_controllers_) {
            if (!controller.needs_replan) continue;

            auto agent_it = std::ranges::find_if(world.agents,
                [&](const auto& a) { return a.id == controller.id; });

            if (agent_it == world.agents.end() || agent_it->at_goal || agent_it->collision_stopped) {
                continue;
            }

            // Clear old reservations and replan
            planner_->clear_reservations(controller.id, controller.local_reservations);

            auto path = planner_->plan_path(
                agent_it->pos,
                agent_it->goal,
                controller.local_reservations,
                controller.id,
                current_tick_
            );

            if (!path.empty()) {
                controller.current_path = path;
                controller.path_index = 0;
                controller.needs_replan = false;
                planner_->commit_reservations(path, controller.id, controller.local_reservations, current_tick_);
            }
        }
    }
}

void Simulation::detect_and_resolve_deadlocks() {
    auto current_tick = world_manager_->get_world().current_tick;

    // Update stuck counters and detect deadlocked agents
    std::vector<boost::uuids::uuid> deadlocked_agents;

    for (auto& controller : agent_controllers_) {
        // Thread-safe access to agent data
        core::AgentState agent_state;
        bool agent_found = false;
        {
            std::lock_guard<std::mutex> lock(world_access_mutex_);
            auto& world = world_manager_->get_world();
            auto agent_it = std::ranges::find_if(world.agents,
                [&](const auto& a) { return a.id == controller.id; });

            if (agent_it != world.agents.end()) {
                agent_state = *agent_it;
                agent_found = true;
            }
        }

        if (!agent_found || agent_state.at_goal) {
            continue;
        }

        // Check if agent has moved since last tick
        if (controller.last_position.x == -1 && controller.last_position.y == -1) {
            // First time tracking this agent
            controller.last_position = agent_state.pos;
            controller.last_successful_move = current_tick;
            controller.stuck_counter = 0;
        } else if (controller.last_position == agent_state.pos) {
            // Agent hasn't moved
            controller.stuck_counter++;

            // More aggressive deadlock detection for collision-stopped agents
            int deadlock_threshold = agent_state.collision_stopped ? 3 : AgentController::DEADLOCK_THRESHOLD;

            if (controller.stuck_counter >= deadlock_threshold) {
                deadlocked_agents.push_back(controller.id);
            }
        } else {
            // Agent moved successfully
            controller.last_position = agent_state.pos;
            controller.last_successful_move = current_tick;
            controller.stuck_counter = 0;
        }
    }

    if (!deadlocked_agents.empty()) {
        if (config_.verbose) {
            spdlog::warn("Deadlock detected with {} agents at tick {}",
                         deadlocked_agents.size(), current_tick);
        }

        // Resolve deadlock by having some agents back off
        resolve_deadlock(deadlocked_agents);
    }
}

void Simulation::resolve_deadlock(const std::vector<boost::uuids::uuid>& deadlocked_agents) {
    auto& world = world_manager_->get_world();

    // Sort agents by ID to have deterministic priority (agents with smaller UUIDs have higher priority)
    std::vector<boost::uuids::uuid> sorted_agents = deadlocked_agents;
    std::sort(sorted_agents.begin(), sorted_agents.end());

    // Have lower priority agents (later in sorted order) back off
    int agents_to_back_off = std::max(1, static_cast<int>(sorted_agents.size() / 2));

    for (int i = sorted_agents.size() - agents_to_back_off; i < sorted_agents.size(); ++i) {
        auto agent_id = sorted_agents[i];
        auto controller_it = std::ranges::find_if(agent_controllers_,
            [&](auto& c) { return c.id == agent_id; });

        if (controller_it != agent_controllers_.end()) {
            // Force the agent to find a new path by clearing current path and setting replan flag
            controller_it->current_path.clear();
            controller_it->path_index = 0;
            controller_it->needs_replan = true;
            controller_it->stuck_counter = 0;  // Reset stuck counter

            // Clear their reservations to give other agents space
            planner_->clear_reservations(agent_id, controller_it->local_reservations);

            // Unfreeze collision-stopped agents to allow them to move
            world_manager_->set_agent_collision_stopped(agent_id, false);

            // Add temporary wait to break the deadlock pattern
            controller_it->wait_counter = 3 + (i % 5);  // Wait 3-7 ticks

            metrics_collector_.record_replan();

            if (config_.verbose) {
                spdlog::debug("Agent {} backing off to resolve deadlock (wait: {} ticks)",
                             boost::uuids::to_string(agent_id), controller_it->wait_counter);
            }
        }
    }
}

void Simulation::execute_moves() {
    // Step 1: Collect all intended moves
    struct IntendedMove {
        boost::uuids::uuid agent_id;
        core::Cell from_pos;
        core::Cell to_pos;
        size_t controller_index;
    };
    std::vector<IntendedMove> intended_moves;

    for (size_t i = 0; i < agent_controllers_.size(); ++i) {
        auto& controller = agent_controllers_[i];

        // Thread-safe access to agent data
        core::AgentState agent_state;
        bool agent_found = false;
        {
            std::lock_guard<std::mutex> lock(world_access_mutex_);
            auto& world = world_manager_->get_world();
            auto agent_it = std::ranges::find_if(world.agents,
                [&](const auto& a) { return a.id == controller.id; });

            if (agent_it != world.agents.end()) {
                agent_state = *agent_it;
                agent_found = true;
            }
        }

        if (!agent_found || agent_state.at_goal || agent_state.collision_stopped) {
            continue;
        }

        if (controller.path_index < controller.current_path.size()) {
            auto next_pos = controller.current_path[controller.path_index];

            // FIX: Removed overly conservative waiting logic that was causing deadlocks.
            // The existing deadlock resolution system is sufficient to handle conflicts.
            // Conservative waiting where all agents wait for information from each other
            // creates a classic distributed deadlock scenario.

            intended_moves.push_back({
                controller.id,
                agent_state.pos,
                next_pos,
                i
            });
        }
    }

    // Step 2: Execute all moves simultaneously (allowing potential collisions)
    std::unordered_map<core::Cell, std::vector<boost::uuids::uuid>, core::CellHash> target_cells;

    for (const auto& move : intended_moves) {
        target_cells[move.to_pos].push_back(move.agent_id);
    }

    // Step 3: Process moves and detect conflicts
    for (const auto& move : intended_moves) {
        auto& controller = agent_controllers_[move.controller_index];

        // Check if this is a conflicting move (multiple agents targeting same cell)
        bool has_conflict = target_cells[move.to_pos].size() > 1;

        // Execute the move with thread-safe access
        {
            std::lock_guard<std::mutex> lock(world_access_mutex_);
            auto& world = world_manager_->get_world();
            auto agent_it = std::ranges::find_if(world.agents,
                [&](auto& a) { return a.id == move.agent_id; });

            if (agent_it != world.agents.end()) {
                // Force the move (bypass occupancy check for simultaneous moves)
                if (world.is_valid_cell(move.to_pos) && world.grid[move.to_pos.y][move.to_pos.x] != '#') {
                    agent_it->pos = move.to_pos;
                    controller.path_index++;
                    controller.last_intent = move.to_pos;

                    if (move.to_pos == agent_it->goal) {
                        agent_it->at_goal = true;
                        spdlog::info("Agent {} reached goal", boost::uuids::to_string(move.agent_id));
                    }

                    // If there's a conflict, agents will need to replan after collision detection
                    if (has_conflict && config_.verbose) {
                        spdlog::debug("Agent {} moved to ({},{}) (potential conflict)",
                                    boost::uuids::to_string(move.agent_id),
                                    move.to_pos.x, move.to_pos.y);
                    }
                } else {
                    // Can't move to invalid/obstacle cell
                    controller.needs_replan = true;
                    metrics_collector_.record_replan();
                }
            }
        }
    }
}

void Simulation::detect_and_handle_collisions() {
    auto colliding_agents = world_manager_->detect_collisions();

    if (!colliding_agents.empty()) {
        spdlog::error("Collision detected at tick {} with {} agents",
                     world_manager_->get_world().current_tick,
                     colliding_agents.size());

        metrics_collector_.record_collision();

        // Enhanced collision resolution with displacement
        for (const auto& agent_id : colliding_agents) {
            auto controller_it = std::ranges::find_if(agent_controllers_,
                [&](auto& c) { return c.id == agent_id; });

            if (controller_it != agent_controllers_.end()) {
                // Try to find an adjacent free cell to displace the agent
                core::Cell current_pos;
                bool found_agent = false;
                {
                    std::lock_guard<std::mutex> lock(world_access_mutex_);
                    auto agent_it = std::ranges::find_if(world_manager_->get_world().agents,
                        [&](const auto& a) { return a.id == agent_id; });
                    if (agent_it != world_manager_->get_world().agents.end()) {
                        current_pos = agent_it->pos;
                        found_agent = true;
                    }
                }

                if (found_agent) {
                    // Try to find a free adjacent cell for displacement
                    std::vector<core::Cell> adjacent_cells = {
                        {current_pos.x + 1, current_pos.y},
                        {current_pos.x - 1, current_pos.y},
                        {current_pos.x, current_pos.y + 1},
                        {current_pos.x, current_pos.y - 1}
                    };

                    bool displaced = false;
                    for (const auto& candidate_pos : adjacent_cells) {
                        if (world_manager_->get_world().is_valid_cell(candidate_pos) &&
                            world_manager_->get_world().grid[candidate_pos.y][candidate_pos.x] != '#' &&
                            !world_manager_->check_collision(agent_id, candidate_pos)) {

                            // Move agent to the free adjacent cell
                            world_manager_->move_agent(agent_id, candidate_pos);
                            displaced = true;

                            if (config_.verbose) {
                                spdlog::info("Displaced colliding agent {} from ({},{}) to ({},{})",
                                           boost::uuids::to_string(agent_id),
                                           current_pos.x, current_pos.y,
                                           candidate_pos.x, candidate_pos.y);
                            }
                            break;
                        }
                    }

                    // If displacement failed, fall back to stopping the agent
                    if (!displaced) {
                        world_manager_->set_agent_collision_stopped(agent_id, true);
                        if (config_.verbose) {
                            spdlog::warn("Could not displace agent {}, marking as collision-stopped",
                                       boost::uuids::to_string(agent_id));
                        }
                    }
                }

                controller_it->needs_replan = true;
                metrics_collector_.record_replan();
            }
        }
    }
}

bool Simulation::check_termination() const {
    return world_manager_->all_agents_at_goal();
}

void Simulation::log_tick_state() const {
    spdlog::debug("Tick {}: {} active agents",
                 world_manager_->get_world().current_tick,
                 world_manager_->count_active_agents());
}

void Simulation::save_outputs() {
    if (!config_.metrics_output.empty()) {
        try {
            emit_metrics_json(config_.metrics_output, metrics_collector_.get_snapshot());
            spdlog::info("Saved metrics to {}", config_.metrics_output.string());
        } catch (const std::exception& e) {
            spdlog::error("Failed to save metrics: {}", e.what());
        }
    }

    if (!config_.trace_output.empty()) {
        try {
            emit_trace_csv(config_.trace_output, metrics_collector_.get_traces());
            spdlog::info("Saved trace to {}", config_.trace_output.string());
        } catch (const std::exception& e) {
            spdlog::error("Failed to save trace: {}", e.what());
        }
    }
}

// GUI interface methods
void Simulation::step() {
    if (!initialized_) {
        if (!initialize()) {
            return;
        }
    }

    if (!is_complete()) {
        step_internal();
        world_manager_->advance_tick();
        current_tick_ = world_manager_->get_world().current_tick;
    }
}

void Simulation::reset() {
    if (!initialized_) {
        return;
    }

    current_tick_ = 0;
    metrics_collector_.reset();
    reservations_.clear();
    network_->reset();

    // Reset world manager
    if (config_.world.has_value()) {
        world_manager_ = core::WorldManager(*config_.world);

        // Reinitialize agent controllers
        agent_controllers_.clear();
        {
            std::lock_guard<std::mutex> lock(world_access_mutex_);
            for (const auto& agent : world_manager_->get_world().agents) {
                AgentController controller;
                controller.id = agent.id;
                controller.last_intent = agent.pos;
                agent_controllers_.push_back(controller);
            }
        }
    }
}

bool Simulation::is_complete() const {
    if (!world_manager_) {
        return false;
    }

    return check_termination() ||
           world_manager_->get_world().current_tick >= config_.max_ticks;
}

const core::World& Simulation::get_world() const {
    return world_manager_->get_world();
}

std::vector<core::AgentState> Simulation::get_agents() const {
    if (!world_manager_) {
        return {};
    }

    std::vector<core::AgentState> agents;
    {
        std::lock_guard<std::mutex> lock(world_access_mutex_);
        agents = world_manager_->get_world().agents;
    }

    // Enhance agent data with planned paths from controllers
    for (auto& agent : agents) {
        for (const auto& controller : agent_controllers_) {
            if (controller.id == agent.id) {
                agent.planned_path = controller.current_path;
                agent.path_index = controller.path_index;
                break;
            }
        }
    }

    return agents;
}

} // namespace swarmgrid