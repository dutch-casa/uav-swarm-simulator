#include <catch2/catch_test_macros.hpp>
#include "swarmgrid/core/planner.hpp"
#include "swarmgrid/core/world.hpp"
#include <ranges>
#include <algorithm>

using namespace swarmgrid::core;

TEST_CASE("ReservationTable operations", "[reservations]") {
    ReservationTable reservations;
    boost::uuids::random_generator gen;
    auto agent1_id = gen();
    auto agent2_id = gen();

    SECTION("Insert and find reservations") {
        ReservationEntry entry{{3, 4, 10}, agent1_id};
        reservations.insert(entry);

        ReservationKey key{3, 4, 10};
        auto it = reservations.find(key);
        REQUIRE(it != reservations.end());
        REQUIRE(it->agent_id == agent1_id);
    }

    SECTION("Multiple agents, no conflicts") {
        ReservationEntry entry1{{1, 1, 5}, agent1_id};
        ReservationEntry entry2{{2, 2, 5}, agent2_id};

        auto [it1, success1] = reservations.insert(entry1);
        auto [it2, success2] = reservations.insert(entry2);

        REQUIRE(success1);
        REQUIRE(success2);
        REQUIRE(reservations.size() == 2);
    }

    SECTION("Conflict detection - same cell, same time") {
        ReservationEntry entry1{{3, 3, 7}, agent1_id};
        ReservationEntry entry2{{3, 3, 7}, agent2_id};

        auto [it1, success1] = reservations.insert(entry1);
        auto [it2, success2] = reservations.insert(entry2);

        REQUIRE(success1);
        REQUIRE(!success2);  // Second insert should fail
        REQUIRE(reservations.size() == 1);
    }

    SECTION("Clear agent reservations using multi-index") {
        // Add multiple reservations for agent1
        for (int t = 0; t < 5; ++t) {
            reservations.insert({{2, 3, t}, agent1_id});
        }

        // Add some for agent2
        for (int t = 0; t < 3; ++t) {
            reservations.insert({{4, 5, t}, agent2_id});
        }

        REQUIRE(reservations.size() == 8);

        // Clear agent1's reservations using the agent_id index
        auto& agent_index = reservations.get<1>();
        agent_index.erase(agent1_id);

        REQUIRE(reservations.size() == 3);

        // Verify only agent2's reservations remain
        for (const auto& entry : reservations) {
            REQUIRE(entry.agent_id == agent2_id);
        }
    }

    SECTION("Reservation hash uniqueness") {
        ReservationHash hasher;

        ReservationKey key1{5, 10, 15};
        ReservationKey key2{5, 10, 15};
        ReservationKey key3{5, 10, 16};
        ReservationKey key4{6, 10, 15};

        REQUIRE(hasher(key1) == hasher(key2));
        REQUIRE(hasher(key1) != hasher(key3));
        REQUIRE(hasher(key1) != hasher(key4));
    }
}

TEST_CASE("Edge collision prevention", "[reservations]") {
    auto world = swarmgrid::core::WorldBuilder(42)
        .with_grid({
            "...",
            "...",
            "..."
        })
        .build();

    REQUIRE(world.has_value());
    PathPlanner planner(*world);
    ReservationTable reservations;

    boost::uuids::random_generator gen;
    auto agent1_id = gen();
    auto agent2_id = gen();

    SECTION("Prevent head-on collision") {
        // Agent 1 moves from (0,1) to (2,1)
        Path path1 = planner.plan_path({0, 1}, {2, 1}, reservations, agent1_id, 0);
        REQUIRE(!path1.empty());
        planner.commit_reservations(path1, agent1_id, reservations, 0);

        // Agent 2 tries to move from (2,1) to (0,1) at the same time
        // This would cause a head-on collision, so the planner should avoid it
        Path path2 = planner.plan_path({2, 1}, {0, 1}, reservations, agent2_id, 0);
        REQUIRE(!path2.empty());

        // The paths should not swap positions at any time step
        bool collision = false;
        for (size_t t = 0; t < std::min(path1.size(), path2.size()) - 1; ++t) {
            if (path1[t] == path2[t+1] && path1[t+1] == path2[t]) {
                collision = true;
                break;
            }
        }
        REQUIRE(!collision);
    }

    SECTION("Goal reservation persistence") {
        // Agent 1 reaches its goal
        Path path1 = planner.plan_path({0, 0}, {2, 2}, reservations, agent1_id, 0);
        REQUIRE(!path1.empty());
        planner.commit_reservations(path1, agent1_id, reservations, 0);

        // Check that goal is reserved for future time steps
        Tick goal_time = static_cast<Tick>(path1.size() - 1);
        for (int future = 0; future < 10; ++future) {
            REQUIRE(planner.is_reserved({2, 2}, goal_time + future, reservations));
        }

        // Agent 2 should not be able to use the goal cell
        Path path2 = planner.plan_path({0, 2}, {2, 2}, reservations, agent2_id, 10);
        bool reaches_occupied_goal = false;
        if (!path2.empty() && path2.back() == Cell{2, 2}) {
            reaches_occupied_goal = true;
        }
        REQUIRE(!reaches_occupied_goal);
    }
}