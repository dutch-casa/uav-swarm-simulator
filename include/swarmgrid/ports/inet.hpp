#pragma once

#include "swarmgrid/core/types.hpp"
#include <boost/uuid/uuid.hpp>
#include <vector>
#include <memory>
#include <concepts>

namespace swarmgrid::ports {

struct Message {
    boost::uuids::uuid from;
    swarmgrid::core::Cell next;
    swarmgrid::core::Tick timestamp;
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