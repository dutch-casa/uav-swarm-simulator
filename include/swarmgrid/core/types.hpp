#pragma once

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/functional/hash.hpp>
#include <vector>
#include <string>
#include <cstdint>
#include <functional>

namespace swarmgrid::core {

struct Cell {
    int x;
    int y;

    bool operator==(const Cell& other) const noexcept {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Cell& other) const noexcept {
        return !(*this == other);
    }

    bool operator<(const Cell& other) const noexcept {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

using Tick = int;

struct AgentState {
    boost::uuids::uuid id;
    Cell pos;
    Cell goal;
    std::vector<Cell> planned_path;
    size_t path_index = 0;
    bool at_goal = false;
    int replans = 0;

    bool operator==(const AgentState& other) const noexcept {
        return id == other.id;
    }
};

struct World {
    int width;
    int height;
    std::vector<std::string> grid;
    std::vector<AgentState> agents;
    uint64_t rng_seed;
    Tick current_tick = 0;

    bool is_valid_cell(const Cell& cell) const noexcept {
        return cell.x >= 0 && cell.x < width &&
               cell.y >= 0 && cell.y < height;
    }

    bool is_free_cell(const Cell& cell) const noexcept {
        if (!is_valid_cell(cell)) return false;
        return grid[cell.y][cell.x] != '#';
    }

    bool is_occupied(const Cell& cell, const boost::uuids::uuid& exclude_id = boost::uuids::nil_uuid()) const noexcept {
        for (const auto& agent : agents) {
            if (agent.id != exclude_id && agent.pos == cell) {
                return true;
            }
        }
        return false;
    }
};

struct CellHash {
    std::size_t operator()(const Cell& cell) const noexcept {
        std::size_t seed = 0;
        boost::hash_combine(seed, cell.x);
        boost::hash_combine(seed, cell.y);
        return seed;
    }
};

// Add hash_value function for boost::hash integration
inline std::size_t hash_value(const Cell& cell) {
    std::size_t seed = 0;
    boost::hash_combine(seed, cell.x);
    boost::hash_combine(seed, cell.y);
    return seed;
}

} // namespace swarmgrid::core