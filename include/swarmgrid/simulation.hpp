#pragma once

#include "swarmgrid/core/world.hpp"
#include "swarmgrid/core/planner.hpp"
#include "swarmgrid/core/metrics.hpp"
#include "swarmgrid/ports/inet.hpp"
#include "swarmgrid/ports/imap_loader.hpp"
#include <memory>
#include <filesystem>
#include <optional>
#include <mutex>

namespace swarmgrid {

struct SimulationConfig {
    std::filesystem::path map_path;
    std::optional<core::World> world;  // Allow direct world specification
    int num_agents = 4;
    uint64_t seed = 42;
    ports::NetworkParams network_params;
    int max_ticks = 1000;
    std::filesystem::path trace_output;
    std::filesystem::path metrics_output;
    bool verbose = false;
};

class Simulation {
public:
    Simulation(SimulationConfig config,
               std::unique_ptr<ports::IMapLoader> map_loader,
               std::unique_ptr<ports::INetwork> network);

    // GUI-friendly constructor
    Simulation(SimulationConfig config,
               std::unique_ptr<ports::INetwork> network);

    bool initialize();
    bool run();

    // GUI interface methods
    void step();
    void reset();
    bool is_complete() const;
    core::Tick get_current_tick() const { return current_tick_; }

    core::MetricsSnapshot get_metrics() const { return metrics_collector_.get_snapshot(); }
    const core::World& get_world() const;
    std::vector<core::AgentState> get_agents() const;

private:
    SimulationConfig config_;
    std::unique_ptr<ports::IMapLoader> map_loader_;
    std::unique_ptr<ports::INetwork> network_;

    std::optional<core::WorldManager> world_manager_;
    std::unique_ptr<core::PathPlanner> planner_;
    core::ReservationTable reservations_;
    core::MetricsCollector metrics_collector_;

    struct AgentController {
        boost::uuids::uuid id;
        core::Path current_path;
        size_t path_index = 0;
        core::Cell last_intent;
        bool needs_replan = true;
        int wait_counter = 0;
        static constexpr int MAX_WAIT = 5;

        // Local view of other agents' intentions
        struct OtherAgentIntent {
            boost::uuids::uuid agent_id;
            core::Cell next_position;
            core::Tick timestamp;
            core::Path announced_path;  // Full path they announced
        };
        std::vector<OtherAgentIntent> known_intents;

        // Mesh-style distributed state management
        core::ReservationTable local_reservations;  // Local view of all reservations
        std::unordered_map<boost::uuids::uuid, uint64_t, boost::hash<boost::uuids::uuid>> last_seen_sequence;  // Track message ordering
        core::Tick last_state_broadcast = 0;  // When we last sent full state
        core::Tick last_state_received = 0;   // When we last received full state from others

        // Vector clock for causal ordering
        std::unordered_map<boost::uuids::uuid, uint64_t, boost::hash<boost::uuids::uuid>> vector_clock;
        uint64_t local_clock = 0;  // This agent's logical clock value

        // State sync frequency control
        static constexpr int STATE_BROADCAST_INTERVAL = 10;  // Broadcast full state every N ticks
        static constexpr int STALE_STATE_THRESHOLD = 15;     // Consider state stale after N ticks

        // Deadlock detection
        int stuck_counter = 0;                               // How many ticks agent hasn't moved
        core::Cell last_position = {-1, -1};               // Track if agent is moving
        core::Tick last_successful_move = 0;               // When agent last successfully moved
        static constexpr int DEADLOCK_THRESHOLD = 6;       // Consider deadlocked after N stuck ticks
    };

    std::vector<AgentController> agent_controllers_;
    core::Tick current_tick_ = 0;
    bool initialized_ = false;

    // Thread synchronization for world access
    mutable std::mutex world_access_mutex_;

    void step_internal();
    void receive_and_update_local_state();
    void plan_agent_moves();
    void broadcast_intents();
    void exchange_intents();  // Keep for compatibility
    void validate_pre_execution_conflicts();
    void detect_and_resolve_deadlocks();
    void resolve_deadlock(const std::vector<boost::uuids::uuid>& deadlocked_agents);
    void execute_moves();
    void detect_and_handle_collisions();
    bool check_termination() const;

    void log_tick_state() const;
    void save_outputs();
};

} // namespace swarmgrid