#include <catch2/catch_test_macros.hpp>
#include "swarmgrid/core/planner.hpp"
#include "swarmgrid/core/world.hpp"

using namespace swarmgrid::core;

TEST_CASE("PathPlanner basic pathfinding", "[planner]") {
    auto world = WorldBuilder(42)
        .with_grid({
            ".....",
            "..#..",
            "..#..",
            ".....",
            "....."
        })
        .build();

    REQUIRE(world.has_value());
    PathPlanner planner(*world);
    ReservationTable reservations;
    boost::uuids::random_generator gen;
    auto agent_id = gen();

    SECTION("Simple path without obstacles") {
        Path path = planner.plan_path({0, 0}, {4, 0}, reservations, agent_id);
        REQUIRE(!path.empty());
        REQUIRE(path.front() == Cell{0, 0});
        REQUIRE(path.back() == Cell{4, 0});
        REQUIRE(path.size() == 5);
    }

    SECTION("Path around obstacles") {
        Path path = planner.plan_path({0, 1}, {4, 1}, reservations, agent_id);
        REQUIRE(!path.empty());
        REQUIRE(path.size() > 5);  // Must go around obstacle
    }

    SECTION("No path to obstacle") {
        Path path = planner.plan_path({0, 0}, {2, 1}, reservations, agent_id);
        REQUIRE(path.empty());
    }

    SECTION("Path from and to same position") {
        Path path = planner.plan_path({0, 0}, {0, 0}, reservations, agent_id);
        REQUIRE(!path.empty());
        REQUIRE(path.size() == 1);
        REQUIRE(path[0] == Cell{0, 0});
    }
}

TEST_CASE("PathPlanner with reservations", "[planner]") {
    auto world = WorldBuilder(42)
        .with_grid({
            ".....",
            ".....",
            ".....",
            ".....",
            "....."
        })
        .build();

    REQUIRE(world.has_value());
    PathPlanner planner(*world);
    ReservationTable reservations;

    boost::uuids::random_generator gen;
    auto agent1_id = gen();
    auto agent2_id = gen();

    SECTION("Avoid reserved cells") {
        // Agent 1 reserves a straight path
        Path path1 = planner.plan_path({0, 2}, {4, 2}, reservations, agent1_id);
        REQUIRE(!path1.empty());
        planner.commit_reservations(path1, agent1_id, reservations);

        // Agent 2 must avoid agent 1's path
        Path path2 = planner.plan_path({2, 0}, {2, 4}, reservations, agent2_id, 2);
        REQUIRE(!path2.empty());

        // Check that agent 2 doesn't collide with agent 1's reserved positions
        for (size_t i = 0; i < path2.size(); ++i) {
            Tick time = 2 + static_cast<Tick>(i);
            REQUIRE(!planner.is_reserved(path2[i], time, reservations, agent2_id));
        }
    }

    SECTION("Clear and recommit reservations") {
        Path path1 = planner.plan_path({0, 0}, {4, 0}, reservations, agent1_id);
        planner.commit_reservations(path1, agent1_id, reservations);

        // Verify reservations exist
        REQUIRE(planner.is_reserved({0, 0}, 0, reservations));

        // Clear reservations
        planner.clear_reservations(agent1_id, reservations);

        // Verify reservations are cleared
        REQUIRE(!planner.is_reserved({0, 0}, 0, reservations));

        // Recommit with new path
        Path path2 = planner.plan_path({1, 1}, {3, 3}, reservations, agent1_id);
        planner.commit_reservations(path2, agent1_id, reservations);
        REQUIRE(planner.is_reserved({1, 1}, 0, reservations));
    }

    SECTION("Wait when path is blocked") {
        // Agent 1 reserves position (2,2) for multiple time steps
        ReservationEntry entry1{{2, 2, 3}, agent1_id};
        ReservationEntry entry2{{2, 2, 4}, agent1_id};
        ReservationEntry entry3{{2, 2, 5}, agent1_id};
        reservations.insert(entry1);
        reservations.insert(entry2);
        reservations.insert(entry3);

        // Agent 2 needs to pass through (2,2)
        Path path = planner.plan_path({0, 2}, {4, 2}, reservations, agent2_id, 0);
        REQUIRE(!path.empty());

        // Path should include waiting or going around
        bool waits_or_avoids = false;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            if (path[i] == path[i+1] || path[i] != Cell{2, 2}) {
                waits_or_avoids = true;
            }
        }
        REQUIRE(waits_or_avoids);
    }
}