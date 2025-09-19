#pragma once

#include "swarmgrid/ports/inet.hpp"
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/functional/hash.hpp>
#include <random>
#include <queue>
#include <unordered_map>
#include <mutex>
#include <chrono>

namespace swarmgrid::adapters {

class NetSimAsio : public swarmgrid::ports::INetwork {
public:
    NetSimAsio(swarmgrid::ports::NetworkParams params, uint64_t seed);
    ~NetSimAsio() override = default;

    void send(const swarmgrid::ports::Message& msg) override;
    std::vector<swarmgrid::ports::Message> receive(
        const boost::uuids::uuid& agent_id,
        swarmgrid::core::Tick current_tick
    ) override;
    void reset() override;

private:
    struct DelayedMessage {
        swarmgrid::ports::Message msg;
        swarmgrid::core::Tick delivery_tick;

        auto operator<=>(const DelayedMessage& other) const {
            return delivery_tick <=> other.delivery_tick;
        }
    };

    swarmgrid::ports::NetworkParams params_;
    std::mt19937 rng_;
    std::uniform_real_distribution<> drop_dist_{0.0, 1.0};
    std::normal_distribution<> latency_dist_;

    mutable std::mutex queue_mutex_;
    std::unordered_map<boost::uuids::uuid,
        std::priority_queue<DelayedMessage,
                          std::vector<DelayedMessage>,
                          std::greater<>>,
        boost::hash<boost::uuids::uuid>> message_queues_;

    uint64_t total_sent_ = 0;
    uint64_t total_dropped_ = 0;

    swarmgrid::core::Tick calculate_delivery_time(swarmgrid::core::Tick send_tick);
    bool should_drop_message();
};

} // namespace swarmgrid::adapters