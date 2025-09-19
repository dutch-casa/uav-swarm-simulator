#pragma once

#include "swarmgrid/core/types.hpp"
#include "swarmgrid/core/metrics.hpp"
#include <boost/uuid/uuid.hpp>
#include <vector>
#include <memory>
#include <concepts>

namespace swarmgrid::ports {

struct RenderState {
    swarmgrid::core::World world;
    std::vector<swarmgrid::core::AgentState> agents;
    swarmgrid::core::MetricsSnapshot metrics;
    swarmgrid::core::Tick current_tick;
    bool simulation_running = false;
    bool simulation_complete = false;
};

class IRenderer {
public:
    virtual ~IRenderer() = default;

    virtual bool initialize() = 0;
    virtual void shutdown() = 0;
    virtual bool should_quit() const = 0;
    virtual void render(const RenderState& state) = 0;
    virtual void present() = 0;

    virtual bool is_paused() const = 0;
    virtual bool step_requested() const = 0;
    virtual bool reset_requested() const = 0;
    virtual float get_speed_multiplier() const = 0;
};

template<typename T>
concept RendererImpl = std::derived_from<T, IRenderer>;

using RendererPtr = std::unique_ptr<IRenderer>;

} // namespace swarmgrid::ports