#pragma once

#include "swarmgrid/ports/imap_loader.hpp"
#include <spdlog/spdlog.h>

namespace swarmgrid::adapters {

class MapLoaderFile : public swarmgrid::ports::IMapLoader {
public:
    MapLoaderFile() = default;
    ~MapLoaderFile() override = default;

    std::optional<core::World> load(
        const std::filesystem::path& path,
        int n_agents,
        uint64_t seed
    ) override;

private:
    std::vector<std::string> read_grid_file(const std::filesystem::path& path) const;
    bool validate_grid(const std::vector<std::string>& grid) const;
};

} // namespace swarmgrid::adapters