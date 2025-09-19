#include <catch2/catch_test_macros.hpp>
#include "swarmgrid/simulation.hpp"
#include "swarmgrid/adapters/map_loader_file.hpp"
#include "swarmgrid/adapters/net_sim_asio.hpp"
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

TEST_CASE("End-to-end simulation", "[integration]") {
    // Create a simple test map
    fs::path test_map = fs::temp_directory_path() / "integration_test.txt";
    std::ofstream file(test_map);
    file << "........\n";
    file << "........\n";
    file << "...##...\n";
    file << "...##...\n";
    file << "........\n";
    file << "........\n";
    file.close();

    SECTION("Basic simulation completes without collisions") {
        swarmgrid::SimulationConfig config;
        config.map_path = test_map;
        config.n_agents = 3;
        config.seed = 42;
        config.network_params.drop_probability = 0.0;
        config.network_params.mean_latency_ms = 0;
        config.network_params.jitter_ms = 0;
        config.max_steps = 100;
        config.trace_output = "";
        config.metrics_output = "";
        config.verbose = false;

        auto map_loader = std::make_unique<swarmgrid::adapters::MapLoaderFile>();
        auto network = std::make_unique<swarmgrid::adapters::NetSimAsio>(
            config.network_params, config.seed);

        swarmgrid::Simulation sim(config, std::move(map_loader), std::move(network));

        REQUIRE(sim.initialize());
        REQUIRE(sim.run());

        auto metrics = sim.get_metrics();
        REQUIRE(!metrics.collision_detected);
        REQUIRE(metrics.makespan > 0);
        REQUIRE(metrics.makespan < 100);
    }

    SECTION("Simulation with network issues still completes") {
        swarmgrid::SimulationConfig config;
        config.map_path = test_map;
        config.n_agents = 2;
        config.seed = 123;
        config.network_params.drop_probability = 0.2;
        config.network_params.mean_latency_ms = 50;
        config.network_params.jitter_ms = 20;
        config.max_steps = 200;
        config.trace_output = "";
        config.metrics_output = "";
        config.verbose = false;

        auto map_loader = std::make_unique<swarmgrid::adapters::MapLoaderFile>();
        auto network = std::make_unique<swarmgrid::adapters::NetSimAsio>(
            config.network_params, config.seed);

        swarmgrid::Simulation sim(config, std::move(map_loader), std::move(network));

        REQUIRE(sim.initialize());
        REQUIRE(sim.run());

        auto metrics = sim.get_metrics();
        REQUIRE(!metrics.collision_detected);
        REQUIRE(metrics.dropped_messages > 0);
        REQUIRE(metrics.total_replans > 0);
    }

    SECTION("Metrics output generation") {
        fs::path metrics_file = fs::temp_directory_path() / "test_metrics.json";
        fs::path trace_file = fs::temp_directory_path() / "test_trace.csv";

        swarmgrid::SimulationConfig config;
        config.map_path = test_map;
        config.n_agents = 2;
        config.seed = 999;
        config.network_params.drop_probability = 0.0;
        config.network_params.mean_latency_ms = 0;
        config.network_params.jitter_ms = 0;
        config.max_steps = 50;
        config.trace_output = trace_file;
        config.metrics_output = metrics_file;
        config.verbose = false;

        auto map_loader = std::make_unique<swarmgrid::adapters::MapLoaderFile>();
        auto network = std::make_unique<swarmgrid::adapters::NetSimAsio>(
            config.network_params, config.seed);

        swarmgrid::Simulation sim(config, std::move(map_loader), std::move(network));

        REQUIRE(sim.initialize());
        REQUIRE(sim.run());

        REQUIRE(fs::exists(metrics_file));
        REQUIRE(fs::exists(trace_file));

        // Check metrics file contains expected fields
        std::ifstream metrics_in(metrics_file);
        std::string metrics_content;
        std::getline(metrics_in, metrics_content);
        REQUIRE(metrics_content.find("\"total_messages\"") != std::string::npos);
        REQUIRE(metrics_content.find("\"makespan\"") != std::string::npos);
        REQUIRE(metrics_content.find("\"collision_detected\"") != std::string::npos);

        // Check trace file has header and data
        std::ifstream trace_in(trace_file);
        std::string header;
        std::getline(trace_in, header);
        REQUIRE(header == "tick,agent_id,x,y,active_agents,messages_sent");

        std::string data_line;
        std::getline(trace_in, data_line);
        REQUIRE(!data_line.empty());

        fs::remove(metrics_file);
        fs::remove(trace_file);
    }

    SECTION("Determinism with same seed") {
        swarmgrid::SimulationConfig config;
        config.map_path = test_map;
        config.n_agents = 3;
        config.seed = 555;
        config.network_params.drop_probability = 0.1;
        config.network_params.mean_latency_ms = 10;
        config.network_params.jitter_ms = 5;
        config.max_steps = 100;
        config.trace_output = "";
        config.metrics_output = "";
        config.verbose = false;

        // Run simulation twice with same seed
        auto map_loader1 = std::make_unique<swarmgrid::adapters::MapLoaderFile>();
        auto network1 = std::make_unique<swarmgrid::adapters::NetSimAsio>(
            config.network_params, config.seed);
        swarmgrid::Simulation sim1(config, std::move(map_loader1), std::move(network1));

        auto map_loader2 = std::make_unique<swarmgrid::adapters::MapLoaderFile>();
        auto network2 = std::make_unique<swarmgrid::adapters::NetSimAsio>(
            config.network_params, config.seed);
        swarmgrid::Simulation sim2(config, std::move(map_loader2), std::move(network2));

        REQUIRE(sim1.initialize());
        REQUIRE(sim1.run());

        REQUIRE(sim2.initialize());
        REQUIRE(sim2.run());

        auto metrics1 = sim1.get_metrics();
        auto metrics2 = sim2.get_metrics();

        REQUIRE(metrics1.makespan == metrics2.makespan);
        REQUIRE(metrics1.dropped_messages == metrics2.dropped_messages);
        REQUIRE(metrics1.collision_detected == metrics2.collision_detected);
    }

    fs::remove(test_map);
}