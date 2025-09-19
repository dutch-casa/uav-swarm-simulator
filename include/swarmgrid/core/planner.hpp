#pragma once

#include "swarmgrid/core/types.hpp"
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <vector>
#include <optional>
#include <unordered_map>

namespace swarmgrid::core {

struct ReservationKey {
    int x;
    int y;
    Tick t;

    bool operator==(const ReservationKey& other) const noexcept {
        return x == other.x && y == other.y && t == other.t;
    }
};

struct ReservationHash {
    std::size_t operator()(const ReservationKey& key) const noexcept;
};

struct ReservationEntry {
    ReservationKey key;
    boost::uuids::uuid agent_id;
};

namespace bmi = boost::multi_index;

using ReservationTable = bmi::multi_index_container<
    ReservationEntry,
    bmi::indexed_by<
        bmi::hashed_unique<
            bmi::member<ReservationEntry, ReservationKey, &ReservationEntry::key>,
            ReservationHash
        >,
        bmi::hashed_non_unique<
            bmi::member<ReservationEntry, boost::uuids::uuid, &ReservationEntry::agent_id>
        >
    >
>;

using Path = std::vector<Cell>;

class PathPlanner {
public:
    explicit PathPlanner(const World& world);

    Path plan_path(
        const Cell& start,
        const Cell& goal,
        const ReservationTable& reservations,
        const boost::uuids::uuid& agent_id,
        Tick start_time = 0
    ) const;

    void commit_reservations(
        const Path& path,
        const boost::uuids::uuid& agent_id,
        ReservationTable& reservations,
        Tick start_time = 0
    ) const;

    void clear_reservations(
        const boost::uuids::uuid& agent_id,
        ReservationTable& reservations
    ) const;

    bool is_reserved(
        const Cell& cell,
        Tick time,
        const ReservationTable& reservations,
        const boost::uuids::uuid& exclude_id = boost::uuids::nil_uuid()
    ) const;

private:
    const World& world_;

    using GridGraph = boost::adjacency_list<
        boost::vecS,
        boost::vecS,
        boost::undirectedS,
        boost::property<boost::vertex_index_t, int>,
        boost::property<boost::edge_weight_t, double>
    >;

    GridGraph graph_;
    std::unordered_map<Cell, size_t, CellHash> cell_to_vertex_;
    std::vector<Cell> vertex_to_cell_;

    void build_graph();
    std::vector<Cell> get_neighbors(const Cell& cell) const;
    double heuristic(const Cell& from, const Cell& to) const;

    Path reconstruct_path(
        const std::unordered_map<Cell, Cell, CellHash>& came_from,
        const Cell& start,
        const Cell& goal
    ) const;

    Path astar_with_reservations(
        const Cell& start,
        const Cell& goal,
        const ReservationTable& reservations,
        const boost::uuids::uuid& agent_id,
        Tick start_time
    ) const;
};

} // namespace swarmgrid::core