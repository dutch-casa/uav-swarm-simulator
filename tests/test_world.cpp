#include <catch2/catch_test_macros.hpp>
#include "swarmgrid/core/world.hpp"

using namespace swarmgrid::core;

TEST_CASE("WorldBuilder", "[world]") {
    SECTION("Build simple world") {
        auto world_opt = WorldBuilder(42)
            .with_grid({
                ".....",
                "..#..",
                ".....",
                "..#..",
                "....."
            })
            .with_agent({0, 0}, {4, 4})
            .build();

        REQUIRE(world_opt.has_value());
        auto& world = *world_opt;

        REQUIRE(world.width == 5);
        REQUIRE(world.height == 5);
        REQUIRE(world.agents.size() == 1);
        REQUIRE(world.agents[0].pos == Cell{0, 0});
        REQUIRE(world.agents[0].goal == Cell{4, 4});
    }

    SECTION("Reject unreachable goals") {
        auto world_opt = WorldBuilder(42)
            .with_grid({
                "..#..",
                "..#..",
                "#####",
                "..#..",
                "..#.."
            })
            .with_agent({0, 0}, {4, 4})
            .build();

        REQUIRE(!world_opt.has_value());
    }

    SECTION("Build with random agents") {
        auto world_opt = WorldBuilder(42)
            .with_grid({
                "........",
                "........",
                "........",
                "........"
            })
            .with_random_agents(3)
            .build();

        REQUIRE(world_opt.has_value());
        REQUIRE(world_opt->agents.size() == 3);

        // Check no agents share start or goal positions
        std::unordered_set<Cell, CellHash> positions;
        for (const auto& agent : world_opt->agents) {
            REQUIRE(positions.find(agent.pos) == positions.end());
            REQUIRE(positions.find(agent.goal) == positions.end());
            positions.insert(agent.pos);
            positions.insert(agent.goal);
        }
    }

    SECTION("Deterministic with seed") {
        auto world1 = WorldBuilder(12345)
            .with_grid({
                "........",
                "........",
                "........",
                "........"
            })
            .with_random_agents(2)
            .build();

        auto world2 = WorldBuilder(12345)
            .with_grid({
                "........",
                "........",
                "........",
                "........"
            })
            .with_random_agents(2)
            .build();

        REQUIRE(world1.has_value());
        REQUIRE(world2.has_value());

        for (size_t i = 0; i < world1->agents.size(); ++i) {
            REQUIRE(world1->agents[i].pos == world2->agents[i].pos);
            REQUIRE(world1->agents[i].goal == world2->agents[i].goal);
        }
    }
}

TEST_CASE("WorldManager", "[world]") {
    auto world = WorldBuilder(42)
        .with_grid({
            ".....",
            "..#..",
            ".....",
            "..#..",
            "....."
        })
        .with_agent({0, 0}, {4, 4})
        .with_agent({4, 0}, {0, 4})
        .build();

    REQUIRE(world.has_value());
    WorldManager manager(std::move(*world));

    SECTION("Move agent valid") {
        auto agent_id = manager.get_world().agents[0].id;
        REQUIRE(manager.move_agent(agent_id, {1, 0}));
        REQUIRE(manager.get_agent_position(agent_id) == Cell{1, 0});
    }

    SECTION("Move agent invalid - obstacle") {
        auto agent_id = manager.get_world().agents[0].id;
        REQUIRE(!manager.move_agent(agent_id, {2, 1}));
    }

    SECTION("Move agent invalid - occupied") {
        auto agent1_id = manager.get_world().agents[0].id;
        auto agent2_id = manager.get_world().agents[1].id;
        manager.move_agent(agent1_id, {1, 0});
        REQUIRE(!manager.move_agent(agent2_id, {1, 0}));
    }

    SECTION("Agent reaches goal") {
        auto agent_id = manager.get_world().agents[0].id;
        manager.move_agent(agent_id, {4, 4});
        REQUIRE(manager.get_world().agents[0].at_goal);
        REQUIRE(!manager.all_agents_at_goal());
        REQUIRE(manager.count_active_agents() == 1);
    }

    SECTION("Collision detection") {
        auto agent1_id = manager.get_world().agents[0].id;
        auto agent2_id = manager.get_world().agents[1].id;

        REQUIRE(manager.detect_collisions().empty());

        manager.move_agent(agent1_id, {2, 2});
        manager.move_agent(agent2_id, {2, 2});

        auto collisions = manager.detect_collisions();
        REQUIRE(collisions.size() == 2);
    }

    SECTION("Check collision before move") {
        auto agent1_id = manager.get_world().agents[0].id;
        auto agent2_id = manager.get_world().agents[1].id;

        manager.move_agent(agent1_id, {2, 2});
        REQUIRE(manager.check_collision(agent2_id, {2, 2}));
        REQUIRE(!manager.check_collision(agent2_id, {3, 3}));
    }
}