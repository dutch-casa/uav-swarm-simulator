#pragma once

#include "swarmgrid/core/world.hpp"
#include <filesystem>
#include <optional>
#include <memory>

namespace swarmgrid::ports {

class IMapLoader {
public:
    virtual ~IMapLoader() = default;

    virtual std::optional<swarmgrid::core::World> load(
        const std::filesystem::path& path,
        int n_agents,
        uint64_t seed
    ) = 0;
};

using MapLoaderPtr = std::unique_ptr<IMapLoader>;

} // namespace swarmgrid::ports