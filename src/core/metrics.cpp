#include "swarmgrid/core/metrics.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>

namespace swarmgrid::core {

void MetricsCollector::record_tick_trace(const TickTrace& trace) {
    std::lock_guard lock(trace_mutex_);
    traces_.push_back(trace);
}

MetricsSnapshot MetricsCollector::get_snapshot() const {
    MetricsSnapshot snapshot;
    snapshot.total_messages = total_messages_.load(std::memory_order_relaxed);
    snapshot.dropped_messages = dropped_messages_.load(std::memory_order_relaxed);
    snapshot.total_replans = total_replans_.load(std::memory_order_relaxed);
    snapshot.collision_detected = collision_detected_.load(std::memory_order_relaxed);
    snapshot.makespan = makespan_;
    snapshot.wall_time = wall_time_;
    return snapshot;
}

std::vector<TickTrace> MetricsCollector::get_traces() const {
    std::lock_guard lock(trace_mutex_);
    return traces_;
}

void MetricsCollector::reset() {
    total_messages_.store(0, std::memory_order_relaxed);
    dropped_messages_.store(0, std::memory_order_relaxed);
    total_replans_.store(0, std::memory_order_relaxed);
    collision_detected_.store(false, std::memory_order_relaxed);
    makespan_ = 0;
    wall_time_ = std::chrono::milliseconds{0};

    std::lock_guard lock(trace_mutex_);
    traces_.clear();
}

void emit_metrics_json(const std::filesystem::path& path, const MetricsSnapshot& metrics) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Failed to open metrics file: " + path.string());
    }

    file << "{\n";
    file << "  \"total_messages\": " << metrics.total_messages << ",\n";
    file << "  \"dropped_messages\": " << metrics.dropped_messages << ",\n";
    file << "  \"total_replans\": " << metrics.total_replans << ",\n";
    file << "  \"makespan\": " << metrics.makespan << ",\n";
    file << "  \"collision_detected\": " << (metrics.collision_detected ? "true" : "false") << ",\n";
    file << "  \"wall_time_ms\": " << metrics.wall_time.count() << ",\n";

    double drop_rate = metrics.total_messages > 0 ?
        static_cast<double>(metrics.dropped_messages) / metrics.total_messages : 0.0;
    file << "  \"drop_rate\": " << std::fixed << std::setprecision(4) << drop_rate << "\n";
    file << "}\n";
}

void emit_trace_csv(const std::filesystem::path& path, const std::vector<TickTrace>& traces) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Failed to open trace file: " + path.string());
    }

    // Write header
    file << "tick,agent_id,x,y,active_agents,messages_sent\n";

    // Write traces
    for (const auto& trace : traces) {
        for (const auto& [agent_id, pos] : trace.agent_positions) {
            file << trace.tick << ","
                 << boost::uuids::to_string(agent_id) << ","
                 << pos.x << ","
                 << pos.y << ","
                 << trace.active_agents << ","
                 << trace.messages_sent << "\n";
        }
    }
}

} // namespace swarmgrid::core