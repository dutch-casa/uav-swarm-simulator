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

    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

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

void ImGuiRenderer::render_grid(const swarmgrid::core::World& world,
                                const std::vector<swarmgrid::core::AgentState>& agents) {
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

    // Draw agents
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
    ImGui::Text("Wall Time: %ld ms", metrics.wall_time.count());

    ImGui::Separator();

    ImGui::Text("Total Messages: %zu", metrics.total_messages);
    ImGui::Text("Dropped Messages: %zu", metrics.dropped_messages);
    ImGui::Text("Drop Rate: %.4f", metrics.total_messages > 0 ?
        static_cast<double>(metrics.dropped_messages) / metrics.total_messages : 0.0);

    ImGui::Separator();

    ImGui::Text("Total Replans: %zu", metrics.total_replans);
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