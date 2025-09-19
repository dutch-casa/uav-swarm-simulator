#include "swarmgrid/core/planner.hpp"
#include <boost/functional/hash.hpp>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>

namespace swarmgrid::core {

std::size_t ReservationHash::operator()(const ReservationKey& key) const noexcept {
    std::size_t seed = 0;
    boost::hash_combine(seed, key.x);
    boost::hash_combine(seed, key.y);
    boost::hash_combine(seed, key.t);
    return seed;
}

PathPlanner::PathPlanner(const World& world) : world_(world) {
    build_graph();
}

void PathPlanner::build_graph() {
    // Create vertices for each free cell
    size_t vertex_id = 0;
    for (int y = 0; y < world_.height; ++y) {
        for (int x = 0; x < world_.width; ++x) {
            Cell cell{x, y};
            if (world_.is_free_cell(cell)) {
                cell_to_vertex_[cell] = vertex_id++;
                vertex_to_cell_.push_back(cell);
            }
        }
    }

    // Create graph with the right number of vertices
    graph_ = GridGraph(vertex_id);

    // Add edges between adjacent free cells
    for (const auto& [cell, vertex] : cell_to_vertex_) {
        auto neighbors = get_neighbors(cell);
        for (const auto& neighbor : neighbors) {
            auto it = cell_to_vertex_.find(neighbor);
            if (it != cell_to_vertex_.end()) {
                boost::add_edge(vertex, it->second, 1.0, graph_);
            }
        }
    }
}

std::vector<Cell> PathPlanner::get_neighbors(const Cell& cell) const {
    std::vector<Cell> neighbors;
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};

    for (int i = 0; i < 4; ++i) {
        Cell next{cell.x + dx[i], cell.y + dy[i]};
        if (world_.is_valid_cell(next) && world_.is_free_cell(next)) {
            neighbors.push_back(next);
        }
    }

    // Also include staying in place as an option
    neighbors.push_back(cell);

    return neighbors;
}

double PathPlanner::heuristic(const Cell& from, const Cell& to) const {
    // Manhattan distance
    return std::abs(from.x - to.x) + std::abs(from.y - to.y);
}

Path PathPlanner::plan_path(
    const Cell& start,
    const Cell& goal,
    const ReservationTable& reservations,
    const boost::uuids::uuid& agent_id,
    Tick start_time
) const {
    if (!world_.is_free_cell(start) || !world_.is_free_cell(goal)) {
        return {};
    }

    return astar_with_reservations(start, goal, reservations, agent_id, start_time);
}

Path PathPlanner::astar_with_reservations(
    const Cell& start,
    const Cell& goal,
    const ReservationTable& reservations,
    const boost::uuids::uuid& agent_id,
    Tick start_time
) const {
    struct State {
        Cell cell;
        Tick time;
        double f_score;

        bool operator>(const State& other) const {
            return f_score > other.f_score;
        }
    };

    std::priority_queue<State, std::vector<State>, std::greater<State>> open_set;
    std::unordered_map<std::pair<Cell, Tick>, double, boost::hash<std::pair<Cell, Tick>>> g_score;
    std::unordered_map<std::pair<Cell, Tick>, std::pair<Cell, Tick>, boost::hash<std::pair<Cell, Tick>>> came_from;

    State start_state{start, start_time, heuristic(start, goal)};
    open_set.push(start_state);
    g_score[{start, start_time}] = 0;

    // Maximum time horizon to prevent infinite loops
    const Tick max_time = start_time + world_.width * world_.height * 2;

    while (!open_set.empty()) {
        State current = open_set.top();
        open_set.pop();

        if (current.cell == goal) {
            // Reconstruct path
            Path path;
            auto key = std::make_pair(current.cell, current.time);

            while (came_from.find(key) != came_from.end()) {
                path.push_back(key.first);
                key = came_from[key];
            }
            path.push_back(start);

            std::reverse(path.begin(), path.end());
            return path;
        }

        if (current.time >= max_time) {
            continue;
        }

        auto neighbors = get_neighbors(current.cell);
        for (const auto& next_cell : neighbors) {
            Tick next_time = current.time + 1;

            // Check if the next cell is reserved at the next time
            if (is_reserved(next_cell, next_time, reservations, agent_id)) {
                continue;
            }

            // Edge collision check: check if another agent is moving into our current position
            if (next_cell != current.cell) {
                bool edge_collision = false;
                for (const auto& entry : reservations) {
                    if (entry.agent_id != agent_id &&
                        entry.key.t == current.time &&
                        entry.key.x == next_cell.x &&
                        entry.key.y == next_cell.y) {

                        // Check if that agent is moving to our current position
                        ReservationKey their_next{current.cell.x, current.cell.y, next_time};
                        auto it = reservations.find(their_next);
                        if (it != reservations.end() && it->agent_id == entry.agent_id) {
                            edge_collision = true;
                            break;
                        }
                    }
                }
                if (edge_collision) {
                    continue;
                }
            }

            double tentative_g = g_score[{current.cell, current.time}] + 1.0;
            auto next_key = std::make_pair(next_cell, next_time);

            if (g_score.find(next_key) == g_score.end() || tentative_g < g_score[next_key]) {
                g_score[next_key] = tentative_g;
                double f = tentative_g + heuristic(next_cell, goal);

                came_from[next_key] = {current.cell, current.time};
                open_set.push({next_cell, next_time, f});
            }
        }
    }

    return {};  // No path found
}

void PathPlanner::commit_reservations(
    const Path& path,
    const boost::uuids::uuid& agent_id,
    ReservationTable& reservations,
    Tick start_time
) const {
    // Clear existing reservations for this agent
    clear_reservations(agent_id, reservations);

    // Add new reservations
    for (size_t i = 0; i < path.size(); ++i) {
        ReservationEntry entry{
            {path[i].x, path[i].y, start_time + static_cast<Tick>(i)},
            agent_id
        };
        reservations.insert(entry);
    }

    // Reserve the goal position for future ticks to prevent others from using it
    if (!path.empty()) {
        const Cell& goal = path.back();
        Tick goal_time = start_time + static_cast<Tick>(path.size());
        for (int future = 0; future < 100; ++future) {
            ReservationEntry entry{
                {goal.x, goal.y, goal_time + future},
                agent_id
            };
            reservations.insert(entry);
        }
    }
}

void PathPlanner::clear_reservations(
    const boost::uuids::uuid& agent_id,
    ReservationTable& reservations
) const {
    auto& agent_index = reservations.get<1>();
    agent_index.erase(agent_id);
}

bool PathPlanner::is_reserved(
    const Cell& cell,
    Tick time,
    const ReservationTable& reservations,
    const boost::uuids::uuid& exclude_id
) const {
    ReservationKey key{cell.x, cell.y, time};
    auto it = reservations.find(key);
    if (it != reservations.end() && it->agent_id != exclude_id) {
        return true;
    }
    return false;
}

Path PathPlanner::reconstruct_path(
    const std::unordered_map<Cell, Cell, CellHash>& came_from,
    const Cell& start,
    const Cell& goal
) const {
    Path path;
    Cell current = goal;

    while (current != start) {
        path.push_back(current);
        auto it = came_from.find(current);
        if (it == came_from.end()) {
            return {};  // Path reconstruction failed
        }
        current = it->second;
    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace swarmgrid::core