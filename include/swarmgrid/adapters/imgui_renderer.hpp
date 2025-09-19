#pragma once

#include "swarmgrid/ports/renderer.hpp"
#include <SDL.h>
#include <memory>

namespace swarmgrid::adapters {

class ImGuiRenderer : public swarmgrid::ports::IRenderer {
public:
    ImGuiRenderer();
    ~ImGuiRenderer() override;

    bool initialize() override;
    void shutdown() override;
    bool should_quit() const override;
    void render(const swarmgrid::ports::RenderState& state) override;
    void present() override;

    bool is_paused() const override { return paused_; }
    bool step_requested() const override;
    bool reset_requested() const override;
    float get_speed_multiplier() const override { return speed_multiplier_; }

private:
    void render_grid(const swarmgrid::core::World& world,
                    const std::vector<swarmgrid::core::AgentState>& agents);
    void render_metrics(const swarmgrid::core::MetricsSnapshot& metrics,
                       swarmgrid::core::Tick current_tick);
    void render_controls();

    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_context_ = nullptr;
    bool quit_requested_ = false;
    bool paused_ = false;
    bool step_requested_flag_ = false;
    bool reset_requested_flag_ = false;
    float speed_multiplier_ = 1.0f;

    // Grid rendering parameters
    static constexpr int CELL_SIZE = 20;
    static constexpr int GRID_PADDING = 50;
};

} // namespace swarmgrid::adapters