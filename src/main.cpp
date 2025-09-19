#include "swarmgrid/simulation.hpp"
#include "swarmgrid/adapters/map_loader_file.hpp"
#include "swarmgrid/adapters/net_sim_asio.hpp"
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <iostream>
#include <filesystem>

namespace po = boost::program_options;
namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    try {
        // Setup logging
        auto console = spdlog::stdout_color_mt("console");
        spdlog::set_default_logger(console);

        // Define command line options
        po::options_description desc("UAV Swarm Coordinator - Options");
        desc.add_options()
            ("help,h", "Show help message")
            ("map,m", po::value<std::string>()->required(), "Path to map file")
            ("agents,n", po::value<int>()->default_value(8), "Number of agents")
            ("seed,s", po::value<uint64_t>()->default_value(1337), "Random seed")
            ("drop,d", po::value<double>()->default_value(0.05), "Message drop probability [0-1]")
            ("latency,l", po::value<int>()->default_value(40), "Mean network latency (ms)")
            ("jitter,j", po::value<int>()->default_value(10), "Network jitter (ms)")
            ("max-steps", po::value<int>()->default_value(300), "Maximum simulation steps")
            ("out-trace", po::value<std::string>()->default_value("trace.csv"), "Output trace CSV file")
            ("out-metrics", po::value<std::string>()->default_value("metrics.json"), "Output metrics JSON file")
            ("verbose,v", "Enable verbose logging")
            ("quiet,q", "Suppress info messages");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << "UAV Swarm Grid Coordinator\n";
            std::cout << "Multi-agent pathfinding with simulated communication\n\n";
            std::cout << desc << "\n";
            std::cout << "Example:\n";
            std::cout << "  ./swarmgrid_app --map maps/demo.txt --agents 8 --seed 1337 \\\n";
            std::cout << "                   --drop 0.05 --latency 40 --jitter 10\n";
            return 0;
        }

        po::notify(vm);

        // Set log level
        if (vm.count("verbose")) {
            spdlog::set_level(spdlog::level::debug);
        } else if (vm.count("quiet")) {
            spdlog::set_level(spdlog::level::warn);
        } else {
            spdlog::set_level(spdlog::level::info);
        }

        // Build configuration
        swarmgrid::SimulationConfig config;
        config.map_path = vm["map"].as<std::string>();
        config.n_agents = vm["agents"].as<int>();
        config.seed = vm["seed"].as<uint64_t>();
        config.network_params.drop_probability = vm["drop"].as<double>();
        config.network_params.mean_latency_ms = vm["latency"].as<int>();
        config.network_params.jitter_ms = vm["jitter"].as<int>();
        config.max_steps = vm["max-steps"].as<int>();
        config.trace_output = vm["out-trace"].as<std::string>();
        config.metrics_output = vm["out-metrics"].as<std::string>();
        config.verbose = vm.count("verbose") > 0;

        // Validate inputs
        if (!fs::exists(config.map_path)) {
            spdlog::error("Map file does not exist: {}", config.map_path.string());
            return 1;
        }

        if (config.n_agents <= 0) {
            spdlog::error("Number of agents must be positive");
            return 1;
        }

        if (config.network_params.drop_probability < 0 || config.network_params.drop_probability > 1) {
            spdlog::error("Drop probability must be between 0 and 1");
            return 1;
        }

        // Create simulation components
        auto map_loader = std::make_unique<swarmgrid::adapters::MapLoaderFile>();
        auto network = std::make_unique<swarmgrid::adapters::NetSimAsio>(
            config.network_params, config.seed);

        // Run simulation
        swarmgrid::Simulation sim(config, std::move(map_loader), std::move(network));

        if (!sim.initialize()) {
            spdlog::error("Failed to initialize simulation");
            return 1;
        }

        spdlog::info("Starting simulation with {} agents, seed {}",
                    config.n_agents, config.seed);
        spdlog::info("Network: drop={:.2f}, latency={}ms, jitter={}ms",
                    config.network_params.drop_probability,
                    config.network_params.mean_latency_ms,
                    config.network_params.jitter_ms);

        if (!sim.run()) {
            spdlog::error("Simulation failed");
            return 1;
        }

        // Print summary
        auto metrics = sim.get_metrics();
        spdlog::info("=== Simulation Results ===");
        spdlog::info("Makespan: {} ticks", metrics.makespan);
        spdlog::info("Total messages: {}", metrics.total_messages);
        spdlog::info("Dropped messages: {} ({:.2f}%)",
                    metrics.dropped_messages,
                    metrics.total_messages > 0 ?
                        100.0 * metrics.dropped_messages / metrics.total_messages : 0.0);
        spdlog::info("Total replans: {}", metrics.total_replans);
        spdlog::info("Collisions: {}", metrics.collision_detected ? "YES" : "NO");
        spdlog::info("Wall time: {}ms", metrics.wall_time.count());

        if (metrics.collision_detected) {
            spdlog::error("COLLISION DETECTED - Simulation failed safety check");
            return 1;
        }

        return 0;

    } catch (const po::error& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << "Use --help for usage information\n";
        return 1;
    } catch (const std::exception& e) {
        spdlog::error("Unexpected error: {}", e.what());
        return 1;
    }
}