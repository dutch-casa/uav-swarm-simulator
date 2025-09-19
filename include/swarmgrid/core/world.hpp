#pragma once

#include "swarmgrid/core/types.hpp"
#include <random>
#include <optional>
#include <unordered_set>

namespace swarmgrid::core {

class WorldBuilder {
public:
    explicit WorldBuilder(uint64_t seed) : rng_(seed), seed_(seed) {}

    WorldBuilder& with_dimensions(int width, int height);
    WorldBuilder& with_grid(std::vector<std::string> grid);
    WorldBuilder& with_random_agents(int n_agents);
    WorldBuilder& with_agent(Cell start, Cell goal);

    std::optional<World> build();

private:
    std::mt19937_64 rng_;
    uint64_t seed_;
    int width_ = 0;
    int height_ = 0;
    std::vector<std::string> grid_;
    std::vector<std::pair<Cell, Cell>> agent_specs_;
    int random_agents_ = 0;

    std::vector<Cell> find_free_cells() const;
    bool is_reachable(const Cell& start, const Cell& goal) const;
};

class WorldManager {
public:
    explicit WorldManager(World world) : world_(std::move(world)) {}

    const World& get_world() const { return world_; }
    World& get_world() { return world_; }

    void advance_tick() { world_.current_tick++; }

    bool move_agent(const boost::uuids::uuid& agent_id, const Cell& new_pos);
    bool all_agents_at_goal() const;
    int count_active_agents() const;

    std::optional<Cell> get_agent_position(const boost::uuids::uuid& agent_id) const;
    std::optional<Cell> get_agent_goal(const boost::uuids::uuid& agent_id) const;

    bool check_collision(const boost::uuids::uuid& agent_id, const Cell& pos) const;
    std::vector<boost::uuids::uuid> detect_collisions() const;

private:
    World world_;
};

} // namespace swarmgrid::core