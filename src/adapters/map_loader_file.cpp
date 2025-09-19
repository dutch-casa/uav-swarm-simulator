#include "swarmgrid/adapters/map_loader_file.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace swarmgrid::adapters {

std::optional<swarmgrid::core::World> MapLoaderFile::load(
    const std::filesystem::path& path,
    int n_agents,
    uint64_t seed
) {
    namespace fs = std::filesystem;

    if (!fs::exists(path)) {
        spdlog::error("Map file does not exist: {}", path.string());
        return std::nullopt;
    }

    auto grid = read_grid_file(path);
    if (grid.empty() || !validate_grid(grid)) {
        spdlog::error("Invalid grid format in file: {}", path.string());
        return std::nullopt;
    }

    spdlog::info("Loaded map {}x{} from {}", grid[0].size(), grid.size(), path.string());

    auto world_opt = swarmgrid::core::WorldBuilder(seed)
        .with_grid(grid)
        .with_random_agents(n_agents)
        .build();

    if (!world_opt) {
        spdlog::error("Failed to build world with {} agents", n_agents);
        return std::nullopt;
    }

    spdlog::info("Created world with {} agents", world_opt->agents.size());
    return world_opt;
}

std::vector<std::string> MapLoaderFile::read_grid_file(const std::filesystem::path& path) const {
    std::vector<std::string> grid;
    std::ifstream file(path);

    if (!file) {
        spdlog::error("Failed to open file: {}", path.string());
        return {};
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '/') {
            continue;
        }

        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (!line.empty()) {
            grid.push_back(line);
        }
    }

    return grid;
}

bool MapLoaderFile::validate_grid(const std::vector<std::string>& grid) const {
    if (grid.empty()) {
        return false;
    }

    size_t width = grid[0].size();
    if (width == 0) {
        return false;
    }

    // Check all rows have same width and valid characters
    for (const auto& row : grid) {
        if (row.size() != width) {
            spdlog::error("Inconsistent row width: expected {}, got {}", width, row.size());
            return false;
        }

        for (char c : row) {
            if (c != '.' && c != '#') {
                spdlog::error("Invalid character in map: '{}'", c);
                return false;
            }
        }
    }

    // Check there are enough free cells for agents
    int free_cells = 0;
    for (const auto& row : grid) {
        free_cells += std::count(row.begin(), row.end(), '.');
    }

    if (free_cells < 2) {
        spdlog::error("Not enough free cells in map: {}", free_cells);
        return false;
    }

    return true;
}

} // namespace swarmgrid::adapters