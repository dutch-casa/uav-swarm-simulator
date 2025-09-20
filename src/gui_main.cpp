#include "swarmgrid/adapters/imgui_renderer.hpp"
#include "swarmgrid/simulation.hpp"
#include "swarmgrid/adapters/net_sim_asio.hpp"
#include "swarmgrid/adapters/map_loader_file.hpp"
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>
#include <iostream>

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description desc("UAV Swarm Grid Coordinator - GUI\nMulti-agent pathfinding with real-time visualization\n\nOptions");
    desc.add_options()
        ("help,h", "Show help message")
        ("map,m", po::value<std::string>(), "Path to map file")
        ("agents,n", po::value<int>()->default_value(8), "Number of agents")
        ("seed,s", po::value<unsigned>()->default_value(1337), "Random seed")
        ("drop,d", po::value<double>()->default_value(0.05), "Message drop probability [0-1]")
        ("latency,l", po::value<int>()->default_value(40), "Mean network latency (ms)")
        ("jitter,j", po::value<int>()->default_value(10), "Network jitter (ms)")
        ("max-steps", po::value<int>()->default_value(300), "Maximum simulation steps")
        ("verbose,v", "Enable verbose logging")
        ("quiet,q", "Suppress info messages");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        std::cout << "\nExample:\n";
        std::cout << "  " << argv[0] << " --map maps/demo.txt --agents 8 --seed 1337 \\\n";
        std::cout << "                   --drop 0.05 --latency 40 --jitter 10" << std::endl;
        return 0;
    }

    if (!vm.count("map")) {
        std::cerr << "Error: Map file is required. Use --help for more information." << std::endl;
        return 1;
    }

    // Configure logging
    if (vm.count("quiet")) {
        spdlog::set_level(spdlog::level::warn);
    } else if (vm.count("verbose")) {
        spdlog::set_level(spdlog::level::debug);
    } else {
        spdlog::set_level(spdlog::level::info);
    }

    try {
        // Initialize renderer
        auto renderer = std::make_unique<swarmgrid::adapters::ImGuiRenderer>();
        if (!renderer->initialize()) {
            spdlog::error("Failed to initialize GUI renderer");
            return 1;
        }

        // Create map loader and load world
        auto map_loader = std::make_unique<swarmgrid::adapters::MapLoaderFile>();

        // Create simulation config
        swarmgrid::SimulationConfig config{
            .map_path = vm["map"].as<std::string>(),
            .num_agents = vm["agents"].as<int>(),
            .seed = vm["seed"].as<unsigned>(),
            .network_params = {
                .drop_probability = vm["drop"].as<double>(),
                .mean_latency_ms = vm["latency"].as<int>(),
                .jitter_ms = vm["jitter"].as<int>()
            },
            .max_ticks = vm["max-steps"].as<int>()
        };

        // Create network
        auto network = std::make_unique<swarmgrid::adapters::NetSimAsio>(config.network_params, config.seed);

        // Create simulation
        swarmgrid::Simulation simulation(config, std::move(map_loader), std::move(network));

        // Initialize the simulation BEFORE the main loop starts
        if (!simulation.initialize()) {
            spdlog::error("Failed to initialize simulation");
            return 1;
        }

        spdlog::info("Starting GUI simulation with {} agents", config.num_agents);

        // Main render loop
        auto last_step_time = std::chrono::steady_clock::now();
        const auto target_step_interval = std::chrono::milliseconds(100); // 10 FPS simulation

        while (!renderer->should_quit()) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = current_time - last_step_time;

            // Handle simulation stepping
            bool should_step = false;
            if (renderer->is_paused()) {
                should_step = renderer->step_requested();
            } else {
                float speed = renderer->get_speed_multiplier();
                auto adjusted_interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                    target_step_interval / speed);
                should_step = elapsed >= adjusted_interval;
            }

            if (renderer->reset_requested()) {
                simulation.reset();
                last_step_time = current_time;
            } else if (should_step && !simulation.is_complete()) {
                simulation.step();
                last_step_time = current_time;
            }

            // Prepare render state
            swarmgrid::ports::RenderState render_state{
                .world = simulation.get_world(),
                .agents = simulation.get_agents(),
                .metrics = simulation.get_metrics(),
                .current_tick = simulation.get_current_tick(),
                .simulation_running = !renderer->is_paused() && !simulation.is_complete(),
                .simulation_complete = simulation.is_complete()
            };

            // Render frame
            renderer->render(render_state);
            renderer->present();

            // Cap framerate to ~60 FPS
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        renderer->shutdown();
        spdlog::info("GUI application terminated normally");

    } catch (const std::exception& e) {
        spdlog::error("Error: {}", e.what());
        return 1;
    }

    return 0;
}