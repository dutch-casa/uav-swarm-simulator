#include <catch2/catch_test_macros.hpp>
#include "swarmgrid/adapters/map_loader_file.hpp"
#include <filesystem>
#include <fstream>

using namespace swarmgrid::adapters;
namespace fs = std::filesystem;

TEST_CASE("MapLoaderFile operations", "[map_loader]") {
    MapLoaderFile loader;

    SECTION("Load valid map file") {
        // Create temporary map file
        fs::path temp_map = fs::temp_directory_path() / "test_map.txt";
        std::ofstream file(temp_map);
        file << ".....\n";
        file << "..#..\n";
        file << ".....\n";
        file << "..#..\n";
        file << ".....\n";
        file.close();

        auto world = loader.load(temp_map, 2, 42);
        REQUIRE(world.has_value());
        REQUIRE(world->width == 5);
        REQUIRE(world->height == 5);
        REQUIRE(world->agents.size() == 2);
        REQUIRE(world->rng_seed == 42);

        fs::remove(temp_map);
    }

    SECTION("Reject non-existent file") {
        fs::path fake_path = "/non/existent/map.txt";
        auto world = loader.load(fake_path, 2, 42);
        REQUIRE(!world.has_value());
    }

    SECTION("Skip comments and empty lines") {
        fs::path temp_map = fs::temp_directory_path() / "test_map_comments.txt";
        std::ofstream file(temp_map);
        file << "// This is a comment\n";
        file << "\n";
        file << "...\n";
        file << "// Another comment\n";
        file << "...\n";
        file << "\n";
        file << "...\n";
        file.close();

        auto world = loader.load(temp_map, 1, 42);
        REQUIRE(world.has_value());
        REQUIRE(world->width == 3);
        REQUIRE(world->height == 3);

        fs::remove(temp_map);
    }

    SECTION("Reject invalid characters") {
        fs::path temp_map = fs::temp_directory_path() / "test_map_invalid.txt";
        std::ofstream file(temp_map);
        file << "..X..\n";  // X is invalid
        file << ".....\n";
        file.close();

        auto world = loader.load(temp_map, 1, 42);
        REQUIRE(!world.has_value());

        fs::remove(temp_map);
    }

    SECTION("Reject inconsistent row widths") {
        fs::path temp_map = fs::temp_directory_path() / "test_map_inconsistent.txt";
        std::ofstream file(temp_map);
        file << ".....\n";
        file << "...\n";  // Different width
        file << ".....\n";
        file.close();

        auto world = loader.load(temp_map, 1, 42);
        REQUIRE(!world.has_value());

        fs::remove(temp_map);
    }

    SECTION("Handle map with too many obstacles") {
        fs::path temp_map = fs::temp_directory_path() / "test_map_blocked.txt";
        std::ofstream file(temp_map);
        file << "###\n";
        file << "#.#\n";
        file << "###\n";
        file.close();

        // Only 1 free cell, can't place agents with start and goal
        auto world = loader.load(temp_map, 1, 42);
        REQUIRE(!world.has_value());

        fs::remove(temp_map);
    }

    SECTION("Deterministic loading with same seed") {
        fs::path temp_map = fs::temp_directory_path() / "test_map_determ.txt";
        std::ofstream file(temp_map);
        file << "........\n";
        file << "........\n";
        file << "........\n";
        file << "........\n";
        file.close();

        auto world1 = loader.load(temp_map, 3, 12345);
        auto world2 = loader.load(temp_map, 3, 12345);

        REQUIRE(world1.has_value());
        REQUIRE(world2.has_value());

        for (size_t i = 0; i < world1->agents.size(); ++i) {
            REQUIRE(world1->agents[i].pos == world2->agents[i].pos);
            REQUIRE(world1->agents[i].goal == world2->agents[i].goal);
        }

        fs::remove(temp_map);
    }
}