#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_perception {

struct Position3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct TargetObservation {
    std::string class_name;
    double confidence = 0.0;
    Position3D position;
    std::int64_t timestamp_ns = 0;
    bool depth_valid = false;
};

enum class TrackingState : std::uint8_t {
    kTentative = 0,
    kConfirmed = 1,
    kLost = 2,
    kProcessed = 3,
};

struct TargetManagerConfig {
    std::size_t confirm_frames = 3U;
    std::size_t lost_frames = 5U;
    double max_match_distance = 0.5;
    double ema_alpha = 0.4;
    double processed_cooldown_sec = 10.0;
};

struct ManagedTarget {
    std::uint32_t target_id = 0U;
    std::string class_name;
    double confidence = 0.0;
    Position3D raw_position;
    Position3D filtered_position;
    std::int64_t first_observation_ns = 0;
    std::int64_t last_observation_ns = 0;
    std::size_t hit_count = 0U;
    std::size_t missed_frames = 0U;
    bool depth_valid = false;
    TrackingState state = TrackingState::kTentative;
    std::int64_t processed_at_ns = 0;
};

class TargetManager {
public:
    explicit TargetManager(TargetManagerConfig config = {});

    const std::vector<ManagedTarget>& Update(const std::vector<TargetObservation>& observations,
                                             std::int64_t update_timestamp_ns);
    bool MarkProcessed(std::uint32_t target_id, std::int64_t timestamp_ns);

    const std::vector<ManagedTarget>& targets() const { return targets_; }
    static bool IsValidObservation(const TargetObservation& observation);

private:
    bool CooldownExpired(const ManagedTarget& target, std::int64_t timestamp_ns) const;
    std::uint32_t AllocateTargetId();
    void UpdateMatchedTarget(ManagedTarget* target, const TargetObservation& observation,
                             std::int64_t update_timestamp_ns);
    void CreateTarget(const TargetObservation& observation);
    void AgeUnmatchedTargets(const std::vector<bool>& matched_targets,
                             std::int64_t update_timestamp_ns);
    void RemoveExpiredTargets(std::int64_t update_timestamp_ns);

    TargetManagerConfig config_;
    std::vector<ManagedTarget> targets_;
    std::uint64_t next_target_id_ = 1U;
    std::int64_t last_update_timestamp_ns_ = -1;
};

const char* TrackingStateName(TrackingState state);

}  // namespace robot_perception
