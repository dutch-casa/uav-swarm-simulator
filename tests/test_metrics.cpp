#include <catch2/catch_test_macros.hpp>
#include "swarmgrid/core/metrics.hpp"
#include <boost/uuid/uuid_generators.hpp>
#include <filesystem>
#include <fstream>
#include <thread>

using namespace swarmgrid::core;

TEST_CASE("MetricsCollector operations", "[metrics]") {
    MetricsCollector collector;

    SECTION("Record individual metrics") {
        collector.record_message_sent();
        collector.record_message_sent();
        collector.record_message_dropped();
        collector.record_replan();
        collector.record_replan();
        collector.record_replan();
        collector.set_makespan(42);

        auto snapshot = collector.get_snapshot();
        REQUIRE(snapshot.total_messages == 2);
        REQUIRE(snapshot.dropped_messages == 1);
        REQUIRE(snapshot.total_replans == 3);
        REQUIRE(snapshot.makespan == 42);
        REQUIRE(!snapshot.collision_detected);
    }

    SECTION("Collision detection") {
        REQUIRE(!collector.get_snapshot().collision_detected);
        collector.record_collision();
        REQUIRE(collector.get_snapshot().collision_detected);
    }

    SECTION("Reset clears all metrics") {
        collector.record_message_sent();
        collector.record_replan();
        collector.record_collision();
        collector.set_makespan(10);

        collector.reset();

        auto snapshot = collector.get_snapshot();
        REQUIRE(snapshot.total_messages == 0);
        REQUIRE(snapshot.total_replans == 0);
        REQUIRE(snapshot.makespan == 0);
        REQUIRE(!snapshot.collision_detected);
    }

    SECTION("Tick traces") {
        boost::uuids::random_generator gen;
        auto agent1 = gen();
        auto agent2 = gen();

        TickTrace trace1{
            0,
            {{agent1, {0, 0}}, {agent2, {5, 5}}},
            2,
            1
        };

        TickTrace trace2{
            1,
            {{agent1, {1, 0}}, {agent2, {4, 5}}},
            2,
            2
        };

        collector.record_tick_trace(trace1);
        collector.record_tick_trace(trace2);

        auto traces = collector.get_traces();
        REQUIRE(traces.size() == 2);
        REQUIRE(traces[0].tick == 0);
        REQUIRE(traces[1].tick == 1);
    }

    SECTION("Wall time measurement") {
        collector.start_timer();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        collector.stop_timer();

        auto snapshot = collector.get_snapshot();
        REQUIRE(snapshot.wall_time.count() >= 10);
        REQUIRE(snapshot.wall_time.count() < 100);  // Should be reasonably fast
    }
}

TEST_CASE("Metrics file output", "[metrics]") {
    namespace fs = std::filesystem;

    SECTION("JSON metrics output") {
        MetricsSnapshot metrics;
        metrics.total_messages = 100;
        metrics.dropped_messages = 5;
        metrics.total_replans = 10;
        metrics.makespan = 25;
        metrics.collision_detected = false;
        metrics.wall_time = std::chrono::milliseconds(1234);

        fs::path temp_file = fs::temp_directory_path() / "test_metrics.json";
        emit_metrics_json(temp_file, metrics);

        REQUIRE(fs::exists(temp_file));

        std::ifstream file(temp_file);
        std::string content;
        std::getline(file, content);

        REQUIRE(content.find("\"total_messages\":100") != std::string::npos);
        REQUIRE(content.find("\"dropped_messages\":5") != std::string::npos);
        REQUIRE(content.find("\"makespan\":25") != std::string::npos);
        REQUIRE(content.find("\"collision_detected\":false") != std::string::npos);

        fs::remove(temp_file);
    }

    SECTION("CSV trace output") {
        boost::uuids::random_generator gen;
        auto agent1 = gen();

        std::vector<TickTrace> traces = {
            {0, {{agent1, {0, 0}}}, 1, 0},
            {1, {{agent1, {1, 0}}}, 1, 1},
            {2, {{agent1, {2, 0}}}, 1, 1}
        };

        fs::path temp_file = fs::temp_directory_path() / "test_trace.csv";
        emit_trace_csv(temp_file, traces);

        REQUIRE(fs::exists(temp_file));

        std::ifstream file(temp_file);
        std::string line;

        // Check header
        std::getline(file, line);
        REQUIRE(line == "tick,agent_id,x,y,active_agents,messages_sent");

        // Check first data line
        std::getline(file, line);
        REQUIRE(line.find("0,") == 0);  // Starts with tick 0
        REQUIRE(line.find(",0,0,") != std::string::npos);  // Contains position 0,0

        fs::remove(temp_file);
    }
}