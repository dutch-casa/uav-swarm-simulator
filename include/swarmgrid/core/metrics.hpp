#pragma once

#include "swarmgrid/core/types.hpp"
#include <string_view>
#include <vector>
#include <chrono>
#include <atomic>
#include <filesystem>

namespace swarmgrid::core {

struct MetricsSnapshot {
    uint64_t total_messages = 0;
    uint64_t dropped_messages = 0;
    uint64_t total_replans = 0;
    Tick makespan = 0;
    bool collision_detected = false;
    std::chrono::milliseconds wall_time{0};
};

struct TickTrace {
    Tick tick;
    std::vector<std::pair<boost::uuids::uuid, Cell>> agent_positions;
    int active_agents;
    int messages_sent;
};

class MetricsCollector {
public:
    MetricsCollector() = default;

    void record_message_sent() { total_messages_.fetch_add(1, std::memory_order_relaxed); }
    void record_message_dropped() { dropped_messages_.fetch_add(1, std::memory_order_relaxed); }
    void record_replan() { total_replans_.fetch_add(1, std::memory_order_relaxed); }
    void record_collision() { collision_detected_.store(true, std::memory_order_relaxed); }
    void set_makespan(Tick makespan) { makespan_ = makespan; }

    void record_tick_trace(const TickTrace& trace);

    MetricsSnapshot get_snapshot() const;
    std::vector<TickTrace> get_traces() const;

    void reset();

    void start_timer() { start_time_ = std::chrono::steady_clock::now(); }
    void stop_timer() {
        auto end_time = std::chrono::steady_clock::now();
        wall_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);
    }

private:
    std::atomic<uint64_t> total_messages_{0};
    std::atomic<uint64_t> dropped_messages_{0};
    std::atomic<uint64_t> total_replans_{0};
    std::atomic<bool> collision_detected_{false};
    Tick makespan_{0};

    mutable std::mutex trace_mutex_;
    std::vector<TickTrace> traces_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::milliseconds wall_time_{0};
};

void emit_metrics_json(const std::filesystem::path& path, const MetricsSnapshot& metrics);
void emit_trace_csv(const std::filesystem::path& path, const std::vector<TickTrace>& traces);

} // namespace swarmgrid::core