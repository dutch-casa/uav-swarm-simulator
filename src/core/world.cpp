#include "swarmgrid/core/world.hpp"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

namespace swarmgrid::core {

WorldBuilder& WorldBuilder::with_dimensions(int width, int height) {
    width_ = width;
    height_ = height;
    return *this;
}

WorldBuilder& WorldBuilder::with_grid(std::vector<std::string> grid) {
    grid_ = std::move(grid);
    if (!grid_.empty()) {
        height_ = static_cast<int>(grid_.size());
        width_ = static_cast<int>(grid_[0].size());
    }
    return *this;
}

WorldBuilder& WorldBuilder::with_random_agents(int n_agents) {
    random_agents_ = n_agents;
    return *this;
}

WorldBuilder& WorldBuilder::with_agent(Cell start, Cell goal) {
    agent_specs_.emplace_back(start, goal);
    return *this;
}

std::vector<Cell> WorldBuilder::find_free_cells() const {
    std::vector<Cell> free_cells;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[y][x] != '#') {
                free_cells.push_back({x, y});
            }
        }
    }
    return free_cells;
}

bool WorldBuilder::is_reachable(const Cell& start, const Cell& goal) const {
    if (grid_[start.y][start.x] == '#' || grid_[goal.y][goal.x] == '#') {
        return false;
    }

    std::queue<Cell> frontier;
    std::unordered_set<Cell, CellHash> visited;

    frontier.push(start);
    visited.insert(start);

    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};

    while (!frontier.empty()) {
        Cell current = frontier.front();
        frontier.pop();

        if (current == goal) {
            return true;
        }

        for (int i = 0; i < 4; ++i) {
            Cell next{current.x + dx[i], current.y + dy[i]};

            if (next.x >= 0 && next.x < width_ &&
                next.y >= 0 && next.y < height_ &&
                grid_[next.y][next.x] != '#' &&
                visited.find(next) == visited.end()) {

                visited.insert(next);
                frontier.push(next);
            }
        }
    }

    return false;
}

std::optional<World> WorldBuilder::build() {
    if (width_ <= 0 || height_ <= 0 || grid_.empty()) {
        return std::nullopt;
    }

    World world;
    world.width = width_;
    world.height = height_;
    world.grid = grid_;
    world.rng_seed = seed_;

    boost::uuids::random_generator uuid_gen;

    // Add specified agents
    for (const auto& [start, goal] : agent_specs_) {
        if (!is_reachable(start, goal)) {
            return std::nullopt;
        }

        AgentState agent;
        agent.id = uuid_gen();
        agent.pos = start;
        agent.goal = goal;
        world.agents.push_back(agent);
    }

    // Add random agents if requested
    if (random_agents_ > 0) {
        auto free_cells = find_free_cells();
        if (free_cells.size() < random_agents_ * 2) {
            return std::nullopt;
        }

        std::shuffle(free_cells.begin(), free_cells.end(), rng_);

        std::unordered_set<Cell, CellHash> used_cells;
        for (const auto& agent : world.agents) {
            used_cells.insert(agent.pos);
            used_cells.insert(agent.goal);
        }

        int added = 0;
        for (size_t i = 0; i < free_cells.size() && added < random_agents_; ++i) {
            for (size_t j = i + 1; j < free_cells.size(); ++j) {
                if (used_cells.find(free_cells[i]) == used_cells.end() &&
                    used_cells.find(free_cells[j]) == used_cells.end() &&
                    is_reachable(free_cells[i], free_cells[j])) {

                    AgentState agent;
                    agent.id = uuid_gen();
                    agent.pos = free_cells[i];
                    agent.goal = free_cells[j];
                    world.agents.push_back(agent);

                    used_cells.insert(free_cells[i]);
                    used_cells.insert(free_cells[j]);

                    ++added;
                    break;
                }
            }
        }

        if (added < random_agents_) {
            return std::nullopt;
        }
    }

    return world;
}

bool WorldManager::move_agent(const boost::uuids::uuid& agent_id, const Cell& new_pos) {
    auto it = std::find_if(world_.agents.begin(), world_.agents.end(),
        [&agent_id](const AgentState& a) { return a.id == agent_id; });

    if (it == world_.agents.end()) {
        return false;
    }

    if (!world_.is_valid_cell(new_pos) || !world_.is_free_cell(new_pos)) {
        return false;
    }

    if (world_.is_occupied(new_pos, agent_id)) {
        return false;
    }

    it->pos = new_pos;
    if (it->pos == it->goal) {
        it->at_goal = true;
    }

    return true;
}

bool WorldManager::all_agents_at_goal() const {
    return std::all_of(world_.agents.begin(), world_.agents.end(),
        [](const AgentState& a) { return a.at_goal; });
}

int WorldManager::count_active_agents() const {
    return std::count_if(world_.agents.begin(), world_.agents.end(),
        [](const AgentState& a) { return !a.at_goal; });
}

std::optional<Cell> WorldManager::get_agent_position(const boost::uuids::uuid& agent_id) const {
    auto it = std::find_if(world_.agents.begin(), world_.agents.end(),
        [&agent_id](const AgentState& a) { return a.id == agent_id; });

    if (it != world_.agents.end()) {
        return it->pos;
    }
    return std::nullopt;
}

std::optional<Cell> WorldManager::get_agent_goal(const boost::uuids::uuid& agent_id) const {
    auto it = std::find_if(world_.agents.begin(), world_.agents.end(),
        [&agent_id](const AgentState& a) { return a.id == agent_id; });

    if (it != world_.agents.end()) {
        return it->goal;
    }
    return std::nullopt;
}

bool WorldManager::check_collision(const boost::uuids::uuid& agent_id, const Cell& pos) const {
    return world_.is_occupied(pos, agent_id);
}

std::vector<boost::uuids::uuid> WorldManager::detect_collisions() const {
    std::vector<boost::uuids::uuid> colliding_agents;
    std::unordered_map<Cell, std::vector<boost::uuids::uuid>, CellHash> position_map;

    for (const auto& agent : world_.agents) {
        position_map[agent.pos].push_back(agent.id);
    }

    for (const auto& [pos, agents] : position_map) {
        if (agents.size() > 1) {
            colliding_agents.insert(colliding_agents.end(), agents.begin(), agents.end());
        }
    }

    return colliding_agents;
}

} // namespace swarmgrid::core