#include <catch2/catch_test_macros.hpp>
#include "swarmgrid/core/types.hpp"

using namespace swarmgrid::core;

TEST_CASE("Cell operations", "[types]") {
    SECTION("Cell equality") {
        Cell c1{5, 10};
        Cell c2{5, 10};
        Cell c3{3, 10};
        Cell c4{5, 7};

        REQUIRE(c1 == c2);
        REQUIRE(c1 != c3);
        REQUIRE(c1 != c4);
    }

    SECTION("Cell comparison for sorting") {
        Cell c1{1, 1};
        Cell c2{1, 2};
        Cell c3{2, 1};

        REQUIRE(c1 < c2);
        REQUIRE(c1 < c3);
        REQUIRE(c2 < c3);
    }

    SECTION("Cell hash") {
        Cell c1{5, 10};
        Cell c2{5, 10};
        Cell c3{10, 5};

        CellHash hasher;
        REQUIRE(hasher(c1) == hasher(c2));
        REQUIRE(hasher(c1) != hasher(c3));
    }
}

TEST_CASE("World validation", "[types]") {
    World world;
    world.width = 10;
    world.height = 10;
    world.grid = std::vector<std::string>(10, std::string(10, '.'));
    world.grid[5][5] = '#';

    SECTION("Cell validity") {
        REQUIRE(world.is_valid_cell({0, 0}));
        REQUIRE(world.is_valid_cell({9, 9}));
        REQUIRE(!world.is_valid_cell({-1, 0}));
        REQUIRE(!world.is_valid_cell({0, -1}));
        REQUIRE(!world.is_valid_cell({10, 0}));
        REQUIRE(!world.is_valid_cell({0, 10}));
    }

    SECTION("Cell freedom") {
        REQUIRE(world.is_free_cell({0, 0}));
        REQUIRE(!world.is_free_cell({5, 5}));
        REQUIRE(!world.is_free_cell({-1, 0}));
    }

    SECTION("Cell occupation") {
        boost::uuids::random_generator gen;
        AgentState agent;
        agent.id = gen();
        agent.pos = {3, 3};
        world.agents.push_back(agent);

        REQUIRE(world.is_occupied({3, 3}));
        REQUIRE(!world.is_occupied({3, 4}));
        REQUIRE(!world.is_occupied({3, 3}, agent.id));
    }
}

TEST_CASE("AgentState", "[types]") {
    boost::uuids::random_generator gen;

    SECTION("Agent equality based on ID") {
        AgentState a1, a2;
        a1.id = gen();
        a2.id = a1.id;
        a1.pos = {0, 0};
        a2.pos = {5, 5};

        REQUIRE(a1 == a2);
    }

    SECTION("Agent initial state") {
        AgentState agent;
        REQUIRE(agent.path_index == 0);
        REQUIRE(!agent.at_goal);
        REQUIRE(agent.replans == 0);
    }
}