#pragma once

#include "swarmgrid/core/world.hpp"
#include "swarmgrid/core/planner.hpp"
#include "swarmgrid/core/metrics.hpp"
#include "swarmgrid/ports/inet.hpp"
#include "swarmgrid/ports/imap_loader.hpp"
#include <memory>
#include <filesystem>
#include <optional>

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
    };

    std::vector<AgentController> agent_controllers_;
    core::Tick current_tick_ = 0;
    bool initialized_ = false;

    void step_internal();
    void plan_agent_moves();
    void exchange_intents();
    void execute_moves();
    void detect_and_handle_collisions();
    bool check_termination() const;

    void log_tick_state() const;
    void save_outputs();
};

} // namespace swarmgrid