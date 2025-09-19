#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "swarmgrid/simulation.hpp"
#include "swarmgrid/adapters/map_loader_file.hpp"
#include "swarmgrid/adapters/net_sim_asio.hpp"
#include <random>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

TEST_CASE("Property: No collisions across random configurations", "[properties]") {
    // Create test map
    fs::path test_map = fs::temp_directory_path() / "property_test.txt";
    std::ofstream file(test_map);
    file << "............\n";
    file << "............\n";
    file << "....####....\n";
    file << "....####....\n";
    file << "............\n";
    file << "............\n";
    file << "............\n";
    file << "............\n";
    file.close();

    SECTION("Random seeds never produce collisions") {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<uint64_t> seed_dist(1, 1000000);
        std::uniform_real_distribution<double> drop_dist(0.0, 0.3);
        std::uniform_int_distribution<int> agent_dist(2, 6);

        for (int trial = 0; trial < 10; ++trial) {
            swarmgrid::SimulationConfig config;
            config.map_path = test_map;
            config.n_agents = agent_dist(gen);
            config.seed = seed_dist(gen);
            config.network_params.drop_probability = drop_dist(gen);
            config.network_params.mean_latency_ms = 0;
            config.network_params.jitter_ms = 0;
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
            INFO("Trial " << trial << " with seed " << config.seed
                 << " and " << config.n_agents << " agents");
            REQUIRE(!metrics.collision_detected);
        }
    }

    SECTION("All agents eventually reach goals") {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<uint64_t> seed_dist(1, 1000000);

        for (int trial = 0; trial < 5; ++trial) {
            swarmgrid::SimulationConfig config;
            config.map_path = test_map;
            config.n_agents = 3;
            config.seed = seed_dist(gen);
            config.network_params.drop_probability = 0.0;
            config.network_params.mean_latency_ms = 0;
            config.network_params.jitter_ms = 0;
            config.max_steps = 500;
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
            INFO("Trial " << trial << " with seed " << config.seed);
            REQUIRE(metrics.makespan < config.max_steps);
        }
    }

    fs::remove(test_map);
}

TEST_CASE("Property: Makespan increases with network delays", "[properties]") {
    fs::path test_map = fs::temp_directory_path() / "delay_test.txt";
    std::ofstream file(test_map);
    file << "......\n";
    file << "......\n";
    file << "......\n";
    file << "......\n";
    file << "......\n";
    file << "......\n";
    file.close();

    uint64_t fixed_seed = 12345;
    int n_agents = 3;

    auto run_with_latency = [&](int latency_ms) -> int {
        swarmgrid::SimulationConfig config;
        config.map_path = test_map;
        config.n_agents = n_agents;
        config.seed = fixed_seed;
        config.network_params.drop_probability = 0.0;
        config.network_params.mean_latency_ms = latency_ms;
        config.network_params.jitter_ms = 0;
        config.max_steps = 500;
        config.trace_output = "";
        config.metrics_output = "";
        config.verbose = false;

        auto map_loader = std::make_unique<swarmgrid::adapters::MapLoaderFile>();
        auto network = std::make_unique<swarmgrid::adapters::NetSimAsio>(
            config.network_params, config.seed);

        swarmgrid::Simulation sim(config, std::move(map_loader), std::move(network));
        sim.initialize();
        sim.run();

        return sim.get_metrics().makespan;
    };

    int makespan_no_delay = run_with_latency(0);
    int makespan_with_delay = run_with_latency(200);

    // Higher latency should generally increase makespan
    REQUIRE(makespan_with_delay >= makespan_no_delay);

    fs::remove(test_map);
}

TEST_CASE("Property: More agents increase complexity", "[properties]") {
    fs::path test_map = fs::temp_directory_path() / "agents_test.txt";
    std::ofstream file(test_map);
    file << "..........\n";
    file << "..........\n";
    file << "..........\n";
    file << "..........\n";
    file << "..........\n";
    file << "..........\n";
    file << "..........\n";
    file << "..........\n";
    file << "..........\n";
    file << "..........\n";
    file.close();

    uint64_t fixed_seed = 99999;

    auto run_with_agents = [&](int n_agents) -> swarmgrid::core::MetricsSnapshot {
        swarmgrid::SimulationConfig config;
        config.map_path = test_map;
        config.n_agents = n_agents;
        config.seed = fixed_seed;
        config.network_params.drop_probability = 0.0;
        config.network_params.mean_latency_ms = 0;
        config.network_params.jitter_ms = 0;
        config.max_steps = 500;
        config.trace_output = "";
        config.metrics_output = "";
        config.verbose = false;

        auto map_loader = std::make_unique<swarmgrid::adapters::MapLoaderFile>();
        auto network = std::make_unique<swarmgrid::adapters::NetSimAsio>(
            config.network_params, config.seed);

        swarmgrid::Simulation sim(config, std::move(map_loader), std::move(network));
        sim.initialize();
        sim.run();

        return sim.get_metrics();
    };

    auto metrics_2 = run_with_agents(2);
    auto metrics_4 = run_with_agents(4);
    auto metrics_6 = run_with_agents(6);

    REQUIRE(!metrics_2.collision_detected);
    REQUIRE(!metrics_4.collision_detected);
    REQUIRE(!metrics_6.collision_detected);

    // More agents should generally mean more messages
    REQUIRE(metrics_4.total_messages >= metrics_2.total_messages);
    REQUIRE(metrics_6.total_messages >= metrics_4.total_messages);

    fs::remove(test_map);
}