#include "swarmgrid/adapters/net_sim_asio.hpp"
#include <algorithm>
#include <ranges>

namespace swarmgrid::adapters {

NetSimAsio::NetSimAsio(swarmgrid::ports::NetworkParams params, uint64_t seed)
    : params_(params)
    , rng_(seed)
    , latency_dist_(static_cast<double>(params.mean_latency_ms),
                    static_cast<double>(params.jitter_ms)) {
}

void NetSimAsio::send(const swarmgrid::ports::Message& msg) {
    std::lock_guard lock(queue_mutex_);

    total_sent_++;

    if (should_drop_message()) {
        total_dropped_++;
        return;
    }

    auto delivery_tick = calculate_delivery_time(msg.timestamp);

    // Broadcast to all agents (in a real swarm, this would use actual network topology)
    // For simulation, we'll need to track all known agents
    // In practice, the simulation will tell us about agents

    DelayedMessage delayed_msg{msg, delivery_tick};

    // Add to all agent queues (broadcast simulation)
    // Note: In the full implementation, we'd track agent IDs from the world
    // For now, we'll use a simplified approach where receive() creates queues on-demand

    // Store in a broadcast queue (we'll refine this based on actual agent discovery)
    message_queues_[boost::uuids::nil_uuid()].push(delayed_msg);
}

std::vector<swarmgrid::ports::Message> NetSimAsio::receive(
    const boost::uuids::uuid& agent_id,
    swarmgrid::core::Tick current_tick
) {
    std::lock_guard lock(queue_mutex_);

    std::vector<swarmgrid::ports::Message> ready_messages;

    // Check broadcast queue
    auto& broadcast_queue = message_queues_[boost::uuids::nil_uuid()];
    std::vector<DelayedMessage> deferred;

    while (!broadcast_queue.empty()) {
        auto delayed_msg = broadcast_queue.top();

        if (delayed_msg.delivery_tick <= current_tick) {
            if (delayed_msg.msg.from != agent_id) {  // Don't receive own messages
                ready_messages.push_back(delayed_msg.msg);
            }
            broadcast_queue.pop();
        } else {
            break;  // Priority queue ensures all later messages have higher delivery times
        }
    }

    // Check agent-specific queue if it exists
    if (auto it = message_queues_.find(agent_id); it != message_queues_.end()) {
        auto& queue = it->second;

        while (!queue.empty()) {
            auto delayed_msg = queue.top();

            if (delayed_msg.delivery_tick <= current_tick) {
                ready_messages.push_back(delayed_msg.msg);
                queue.pop();
            } else {
                break;
            }
        }
    }

    return ready_messages;
}

void NetSimAsio::reset() {
    std::lock_guard lock(queue_mutex_);
    message_queues_.clear();
    total_sent_ = 0;
    total_dropped_ = 0;
}

swarmgrid::core::Tick NetSimAsio::calculate_delivery_time(swarmgrid::core::Tick send_tick) {
    if (params_.mean_latency_ms == 0 && params_.jitter_ms == 0) {
        return send_tick + 1;  // Immediate delivery on next tick
    }

    double latency_ms = std::max(0.0, latency_dist_(rng_));

    // Convert ms to ticks (assuming 100ms per tick as a baseline)
    constexpr int MS_PER_TICK = 100;
    int latency_ticks = static_cast<int>(latency_ms / MS_PER_TICK) + 1;

    return send_tick + latency_ticks;
}

bool NetSimAsio::should_drop_message() {
    return drop_dist_(rng_) < params_.drop_probability;
}

swarmgrid::ports::NetworkStats NetSimAsio::get_stats() const {
    std::lock_guard lock(queue_mutex_);
    return {total_sent_, total_dropped_};
}

} // namespace swarmgrid::adapters