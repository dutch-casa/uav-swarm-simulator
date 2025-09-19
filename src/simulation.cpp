#include "swarmgrid/simulation.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <ranges>

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
    for (const auto& agent : world_manager_->get_world().agents) {
        AgentController controller;
        controller.id = agent.id;
        controller.last_intent = agent.pos;
        agent_controllers_.push_back(controller);
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

    plan_agent_moves();
    exchange_intents();
    execute_moves();
    detect_and_handle_collisions();

    // Record trace
    core::TickTrace trace;
    trace.tick = world_manager_->get_world().current_tick;
    trace.active_agents = world_manager_->count_active_agents();
    trace.messages_sent = 0;  // Will be updated in exchange_intents

    for (const auto& agent : world_manager_->get_world().agents) {
        trace.agent_positions.emplace_back(agent.id, agent.pos);
    }

    metrics_collector_.record_tick_trace(trace);
}

void Simulation::plan_agent_moves() {
    auto& world = world_manager_->get_world();

    for (auto& controller : agent_controllers_) {
        auto agent_it = std::ranges::find_if(world.agents,
            [&](const auto& a) { return a.id == controller.id; });

        if (agent_it == world.agents.end() || agent_it->at_goal) {
            continue;
        }

        if (controller.needs_replan || controller.current_path.empty() ||
            controller.path_index >= controller.current_path.size()) {

            // Clear old reservations and plan new path
            planner_->clear_reservations(controller.id, reservations_);

            auto path = planner_->plan_path(
                agent_it->pos,
                agent_it->goal,
                reservations_,
                controller.id,
                world.current_tick
            );

            if (!path.empty()) {
                controller.current_path = path;
                controller.path_index = 0;
                controller.needs_replan = false;
                controller.wait_counter = 0;

                planner_->commit_reservations(path, controller.id, reservations_, world.current_tick);

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
    }
}

void Simulation::exchange_intents() {
    // Broadcast intents through network
    for (const auto& controller : agent_controllers_) {
        if (controller.path_index < controller.current_path.size()) {
            ports::Message msg{
                controller.id,
                controller.current_path[controller.path_index],
                world_manager_->get_world().current_tick
            };
            network_->send(msg);
            metrics_collector_.record_message_sent();
        }
    }

    // Receive intents from other agents
    for (auto& controller : agent_controllers_) {
        auto messages = network_->receive(controller.id, world_manager_->get_world().current_tick);

        for (const auto& msg : messages) {
            // Check if received intent conflicts with our plan
            if (controller.path_index < controller.current_path.size()) {
                auto our_next = controller.current_path[controller.path_index];

                if (msg.next == our_next) {
                    // Conflict detected, may need to replan
                    if (msg.from < controller.id) {  // Simple priority: lower UUID wins
                        controller.needs_replan = true;
                        metrics_collector_.record_replan();
                    }
                }
            }
        }
    }
}

void Simulation::execute_moves() {
    auto& world = world_manager_->get_world();

    for (auto& controller : agent_controllers_) {
        auto agent_it = std::ranges::find_if(world.agents,
            [&](auto& a) { return a.id == controller.id; });

        if (agent_it == world.agents.end() || agent_it->at_goal) {
            continue;
        }

        if (controller.path_index < controller.current_path.size()) {
            auto next_pos = controller.current_path[controller.path_index];

            // Try to move
            if (world_manager_->move_agent(controller.id, next_pos)) {
                controller.path_index++;
                controller.last_intent = next_pos;

                if (next_pos == agent_it->goal) {
                    agent_it->at_goal = true;
                    spdlog::info("Agent {} reached goal", boost::uuids::to_string(controller.id));
                }
            } else {
                // Move failed, need to replan
                controller.needs_replan = true;
                metrics_collector_.record_replan();
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

        // Mark colliding agents for replanning
        for (const auto& agent_id : colliding_agents) {
            auto it = std::ranges::find_if(agent_controllers_,
                [&](auto& c) { return c.id == agent_id; });

            if (it != agent_controllers_.end()) {
                it->needs_replan = true;
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
        for (const auto& agent : world_manager_->get_world().agents) {
            AgentController controller;
            controller.id = agent.id;
            controller.last_intent = agent.pos;
            agent_controllers_.push_back(controller);
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
    return world_manager_->get_world().agents;
}

} // namespace swarmgrid