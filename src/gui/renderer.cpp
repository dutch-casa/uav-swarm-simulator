#include "swarmgrid/adapters/imgui_renderer.hpp"
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>
#include <SDL_opengl.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <spdlog/spdlog.h>
#include <boost/uuid/uuid_io.hpp>
#include <algorithm>
#include <unordered_map>

namespace swarmgrid::adapters {

ImGuiRenderer::ImGuiRenderer() = default;

ImGuiRenderer::~ImGuiRenderer() {
    shutdown();
}

bool ImGuiRenderer::initialize() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        spdlog::error("Failed to initialize SDL: {}", SDL_GetError());
        return false;
    }

    // GL 3.2 Core + GLSL 150 for macOS compatibility
    const char* glsl_version = "#version 150";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    SDL_WindowFlags window_flags = static_cast<SDL_WindowFlags>(
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI
    );

    window_ = SDL_CreateWindow(
        "UAV Swarm Grid Coordinator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1280, 720,
        window_flags
    );

    if (!window_) {
        spdlog::error("Failed to create SDL window: {}", SDL_GetError());
        return false;
    }

    gl_context_ = SDL_GL_CreateContext(window_);
    SDL_GL_MakeCurrent(window_, gl_context_);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window_, gl_context_);
    ImGui_ImplOpenGL3_Init(glsl_version);

    spdlog::info("GUI initialized successfully");
    return true;
}

void ImGuiRenderer::shutdown() {
    if (gl_context_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();

        SDL_GL_DeleteContext(gl_context_);
        gl_context_ = nullptr;
    }

    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }

    SDL_Quit();
}

bool ImGuiRenderer::should_quit() const {
    return quit_requested_;
}

bool ImGuiRenderer::step_requested() const {
    bool result = step_requested_flag_;
    const_cast<ImGuiRenderer*>(this)->step_requested_flag_ = false;
    return result;
}

bool ImGuiRenderer::reset_requested() const {
    bool result = reset_requested_flag_;
    if (result) {
        const_cast<ImGuiRenderer*>(this)->reset_visualization();
    }
    const_cast<ImGuiRenderer*>(this)->reset_requested_flag_ = false;
    return result;
}

void ImGuiRenderer::render(const swarmgrid::ports::RenderState& state) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT) {
            quit_requested_ = true;
        }
    }

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // Clear background
    glViewport(0, 0, static_cast<int>(ImGui::GetIO().DisplaySize.x), static_cast<int>(ImGui::GetIO().DisplaySize.y));
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Render GUI components
    render_grid(state.world, state.agents);
    render_metrics(state.metrics, state.current_tick);
    render_controls();

    // Render Dear ImGui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiRenderer::present() {
    SDL_GL_SwapWindow(window_);
}

void ImGuiRenderer::update_agent_trails(const std::vector<swarmgrid::core::AgentState>& agents) {
    for (const auto& agent : agents) {
        // Track agent movement for trail visualization
        auto it = last_agent_positions_.find(agent.id);
        if (it != last_agent_positions_.end()) {
            // Agent moved, add previous position to trail
            if (it->second != agent.pos) {
                agent_trails_[agent.id].push_back(it->second);
                // Limit trail length to prevent memory growth
                if (agent_trails_[agent.id].size() > 100) {
                    agent_trails_[agent.id].erase(agent_trails_[agent.id].begin());
                }
            }
        }
        last_agent_positions_[agent.id] = agent.pos;
    }

    // Detect collisions for visualization
    std::unordered_map<swarmgrid::core::Cell, std::vector<boost::uuids::uuid>, swarmgrid::core::CellHash> position_map;
    for (const auto& agent : agents) {
        position_map[agent.pos].push_back(agent.id);
    }

    for (const auto& [pos, agent_ids] : position_map) {
        if (agent_ids.size() > 1) {
            collision_locations_.insert(pos);
        }
    }
}

void ImGuiRenderer::reset_visualization() {
    agent_trails_.clear();
    collision_locations_.clear();
    last_agent_positions_.clear();
}

void ImGuiRenderer::render_grid(const swarmgrid::core::World& world,
                                const std::vector<swarmgrid::core::AgentState>& agents) {
    // Update trail tracking
    update_agent_trails(agents);

    ImGui::Begin("Grid Visualization", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();

    // Calculate grid dimensions
    int grid_width = world.width * CELL_SIZE;
    int grid_height = world.height * CELL_SIZE;

    // Draw grid background
    ImU32 bg_color = IM_COL32(50, 50, 50, 255);
    draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + grid_width, canvas_pos.y + grid_height), bg_color);

    // Draw grid lines
    ImU32 line_color = IM_COL32(100, 100, 100, 255);
    for (int x = 0; x <= world.width; ++x) {
        float px = canvas_pos.x + x * CELL_SIZE;
        draw_list->AddLine(ImVec2(px, canvas_pos.y), ImVec2(px, canvas_pos.y + grid_height), line_color);
    }
    for (int y = 0; y <= world.height; ++y) {
        float py = canvas_pos.y + y * CELL_SIZE;
        draw_list->AddLine(ImVec2(canvas_pos.x, py), ImVec2(canvas_pos.x + grid_width, py), line_color);
    }

    // Draw obstacles
    ImU32 obstacle_color = IM_COL32(139, 69, 19, 255); // Brown
    for (int y = 0; y < world.height; ++y) {
        for (int x = 0; x < world.width; ++x) {
            if (world.grid[y][x] == '#') {
                ImVec2 top_left(canvas_pos.x + x * CELL_SIZE + 1, canvas_pos.y + y * CELL_SIZE + 1);
                ImVec2 bottom_right(canvas_pos.x + (x + 1) * CELL_SIZE - 1, canvas_pos.y + (y + 1) * CELL_SIZE - 1);
                draw_list->AddRectFilled(top_left, bottom_right, obstacle_color);
            }
        }
    }

    // Generate consistent colors for agents
    static std::unordered_map<boost::uuids::uuid, ImU32, boost::hash<boost::uuids::uuid>> agent_colors;

    // Draw agent trails (opaque path of visited cells)
    for (const auto& [agent_id, trail] : agent_trails_) {
        if (agent_colors.find(agent_id) == agent_colors.end()) continue;

        ImU32 trail_color = agent_colors[agent_id];
        // Make trail slightly darker than agent color
        ImU32 dark_trail_color = IM_COL32(
            static_cast<int>((trail_color & 0xFF0000) >> 16) * 0.7,
            static_cast<int>((trail_color & 0x00FF00) >> 8) * 0.7,
            static_cast<int>(trail_color & 0x0000FF) * 0.7,
            255
        );

        for (const auto& cell : trail) {
            ImVec2 top_left(canvas_pos.x + cell.x * CELL_SIZE + 2, canvas_pos.y + cell.y * CELL_SIZE + 2);
            ImVec2 bottom_right(canvas_pos.x + (cell.x + 1) * CELL_SIZE - 2, canvas_pos.y + (cell.y + 1) * CELL_SIZE - 2);
            draw_list->AddRectFilled(top_left, bottom_right, dark_trail_color);
        }
    }

    // Draw collision markers (red X)
    for (const auto& collision_pos : collision_locations_) {
        ImVec2 center(
            canvas_pos.x + collision_pos.x * CELL_SIZE + CELL_SIZE / 2,
            canvas_pos.y + collision_pos.y * CELL_SIZE + CELL_SIZE / 2
        );

        ImU32 red_color = IM_COL32(255, 0, 0, 255);
        float size = CELL_SIZE * 0.4f;

        // Draw X mark
        draw_list->AddLine(
            ImVec2(center.x - size, center.y - size),
            ImVec2(center.x + size, center.y + size),
            red_color, 3.0f
        );
        draw_list->AddLine(
            ImVec2(center.x - size, center.y + size),
            ImVec2(center.x + size, center.y - size),
            red_color, 3.0f
        );
    }

    // Draw planned paths (faded)
    for (const auto& agent : agents) {
        if (agent_colors.find(agent.id) == agent_colors.end()) {
            // Generate a color based on agent ID hash
            auto hash = boost::hash<boost::uuids::uuid>{}(agent.id);
            ImU32 color = IM_COL32(
                static_cast<int>((hash & 0xFF0000) >> 16) | 0x80,
                static_cast<int>((hash & 0x00FF00) >> 8) | 0x80,
                static_cast<int>(hash & 0x0000FF) | 0x80,
                255
            );
            agent_colors[agent.id] = color;
        }

        ImU32 agent_color = agent_colors[agent.id];

        // Draw planned path (faded)
        if (!agent.planned_path.empty()) {
            ImU32 faded_color = IM_COL32(
                static_cast<int>((agent_color & 0xFF0000) >> 16),
                static_cast<int>((agent_color & 0x00FF00) >> 8),
                static_cast<int>(agent_color & 0x0000FF),
                80  // Low alpha for faded effect
            );

            // Draw path from current position to path index onwards
            for (size_t i = agent.path_index; i < agent.planned_path.size(); ++i) {
                const auto& cell = agent.planned_path[i];
                ImVec2 center(
                    canvas_pos.x + cell.x * CELL_SIZE + CELL_SIZE / 2,
                    canvas_pos.y + cell.y * CELL_SIZE + CELL_SIZE / 2
                );

                // Draw small circle for planned path
                draw_list->AddCircleFilled(center, CELL_SIZE * 0.15f, faded_color);

                // Draw line to next cell
                if (i < agent.planned_path.size() - 1) {
                    const auto& next_cell = agent.planned_path[i + 1];
                    ImVec2 next_center(
                        canvas_pos.x + next_cell.x * CELL_SIZE + CELL_SIZE / 2,
                        canvas_pos.y + next_cell.y * CELL_SIZE + CELL_SIZE / 2
                    );
                    draw_list->AddLine(center, next_center, faded_color, 2.0f);
                }
            }
        }
    }

    // Draw agents
    for (const auto& agent : agents) {
        ImU32 agent_color = agent_colors[agent.id];
        ImVec2 center(
            canvas_pos.x + agent.pos.x * CELL_SIZE + CELL_SIZE / 2,
            canvas_pos.y + agent.pos.y * CELL_SIZE + CELL_SIZE / 2
        );

        // Draw agent as circle
        draw_list->AddCircleFilled(center, CELL_SIZE * 0.3f, agent_color);
        draw_list->AddCircle(center, CELL_SIZE * 0.3f, IM_COL32(255, 255, 255, 255), 12, 2.0f);

        // Draw goal if different from position
        if (agent.goal != agent.pos) {
            ImVec2 goal_center(
                canvas_pos.x + agent.goal.x * CELL_SIZE + CELL_SIZE / 2,
                canvas_pos.y + agent.goal.y * CELL_SIZE + CELL_SIZE / 2
            );
            draw_list->AddCircle(goal_center, CELL_SIZE * 0.2f, agent_color, 12, 3.0f);
        }
    }

    // Set window size to fit grid
    ImGui::SetWindowSize(ImVec2(grid_width + 20, grid_height + 60));
    ImGui::Dummy(ImVec2(grid_width, grid_height));

    ImGui::End();
}

void ImGuiRenderer::render_metrics(const swarmgrid::core::MetricsSnapshot& metrics,
                                   swarmgrid::core::Tick current_tick) {
    ImGui::Begin("Simulation Metrics");

    ImGui::Text("Current Tick: %d", current_tick);
    ImGui::Text("Makespan: %d", metrics.makespan);
    ImGui::Text("Wall Time: %lld ms", static_cast<long long>(metrics.wall_time.count()));

    ImGui::Separator();

    ImGui::Text("Total Messages: %llu", static_cast<unsigned long long>(metrics.total_messages));
    ImGui::Text("Dropped Messages: %llu", static_cast<unsigned long long>(metrics.dropped_messages));
    ImGui::Text("Drop Rate: %.4f", metrics.total_messages > 0 ?
        static_cast<double>(metrics.dropped_messages) / metrics.total_messages : 0.0);

    ImGui::Separator();

    ImGui::Text("Total Replans: %llu", static_cast<unsigned long long>(metrics.total_replans));
    ImGui::Text("Collision Detected: %s", metrics.collision_detected ? "Yes" : "No");

    ImGui::End();
}

void ImGuiRenderer::render_controls() {
    ImGui::Begin("Simulation Controls");

    if (ImGui::Button(paused_ ? "Resume" : "Pause")) {
        paused_ = !paused_;
    }

    ImGui::SameLine();
    if (ImGui::Button("Step") && paused_) {
        step_requested_flag_ = true;
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        reset_requested_flag_ = true;
    }

    ImGui::Separator();

    ImGui::SliderFloat("Speed", &speed_multiplier_, 0.1f, 5.0f, "%.1fx");

    ImGui::End();
}

} // namespace swarmgrid::adapters