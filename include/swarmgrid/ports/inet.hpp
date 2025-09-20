#pragma once

#include "swarmgrid/core/types.hpp"
#include "swarmgrid/core/planner.hpp"  // For Path and ReservationTable types
#include <boost/uuid/uuid.hpp>
#include <vector>
#include <memory>
#include <concepts>

namespace swarmgrid::ports {

// ReservationTable is defined in planner.hpp which is already included

enum class MessageType {
    PATH_ANNOUNCEMENT,    // Agent announcing its planned path
    STATE_SYNC,          // Full reservation table sync
    GOAL_REACHED         // Agent announcing permanent goal occupation (high priority)
};

struct Message {
    boost::uuids::uuid from;
    MessageType type = MessageType::PATH_ANNOUNCEMENT;
    swarmgrid::core::Cell next;
    swarmgrid::core::Tick timestamp;
    swarmgrid::core::Path planned_path;  // Full planned path for coordination

    // For mesh-style state sharing
    uint64_t sequence_number = 0;  // For ordering messages
    std::shared_ptr<swarmgrid::core::ReservationTable> full_state = nullptr;  // Complete reservation table

    // Vector clock for causal ordering of events
    std::unordered_map<boost::uuids::uuid, uint64_t, boost::hash<boost::uuids::uuid>> vector_clock;
};

struct NetworkParams {
    double drop_probability = 0.0;
    int mean_latency_ms = 0;
    int jitter_ms = 0;
};

struct NetworkStats {
    uint64_t sent = 0;
    uint64_t dropped = 0;
};

class INetwork {
public:
    virtual ~INetwork() = default;

    virtual void send(const Message& msg) = 0;
    virtual std::vector<Message> receive(const boost::uuids::uuid& agent_id, swarmgrid::core::Tick current_tick) = 0;
    virtual void reset() = 0;
    virtual NetworkStats get_stats() const = 0;
};

template<typename T>
concept NetworkImpl = std::derived_from<T, INetwork>;

using NetworkPtr = std::unique_ptr<INetwork>;

} // namespace swarmgrid::ports