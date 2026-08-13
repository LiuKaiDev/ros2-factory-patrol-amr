#include "robot_perception/target_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robot_perception {
namespace {

struct AssociationCandidate {
    double distance = 0.0;
    std::size_t target_index = 0U;
    std::size_t observation_index = 0U;
    std::uint32_t target_id = 0U;
};

bool IsFinitePosition(const Position3D& position) {
    return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
}

double Distance(const Position3D& left, const Position3D& right) {
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Position3D FilteredPosition(const Position3D& previous, const Position3D& current,
                            const double alpha) {
    return {
        alpha * current.x + (1.0 - alpha) * previous.x,
        alpha * current.y + (1.0 - alpha) * previous.y,
        alpha * current.z + (1.0 - alpha) * previous.z,
    };
}

}  // namespace

const char* TrackingStateName(const TrackingState state) {
    switch (state) {
        case TrackingState::kTentative:
            return "TENTATIVE";
        case TrackingState::kConfirmed:
            return "CONFIRMED";
        case TrackingState::kLost:
            return "LOST";
        case TrackingState::kProcessed:
            return "PROCESSED";
    }
    return "UNKNOWN";
}

TargetManager::TargetManager(TargetManagerConfig config) : config_(config) {
    if (config_.confirm_frames == 0U) {
        throw std::invalid_argument("confirm_frames must be positive");
    }
    if (config_.lost_frames == 0U) {
        throw std::invalid_argument("lost_frames must be positive");
    }
    if (!std::isfinite(config_.max_match_distance) || config_.max_match_distance <= 0.0) {
        throw std::invalid_argument("max_match_distance must be positive and finite");
    }
    if (!std::isfinite(config_.ema_alpha) || config_.ema_alpha <= 0.0 || config_.ema_alpha > 1.0) {
        throw std::invalid_argument("ema_alpha must be finite and in (0, 1]");
    }
    if (!std::isfinite(config_.processed_cooldown_sec) || config_.processed_cooldown_sec < 0.0) {
        throw std::invalid_argument("processed_cooldown_sec must be nonnegative and finite");
    }
}

bool TargetManager::IsValidObservation(const TargetObservation& observation) {
    return !observation.class_name.empty() && observation.depth_valid &&
           observation.timestamp_ns >= 0 && std::isfinite(observation.confidence) &&
           observation.confidence >= 0.0 && observation.confidence <= 1.0 &&
           IsFinitePosition(observation.position);
}

bool TargetManager::CooldownExpired(const ManagedTarget& target,
                                    const std::int64_t timestamp_ns) const {
    if (target.state != TrackingState::kProcessed || timestamp_ns < target.processed_at_ns) {
        return false;
    }
    const double elapsed_sec = static_cast<double>(timestamp_ns - target.processed_at_ns) * 1.0e-9;
    return elapsed_sec >= config_.processed_cooldown_sec;
}

std::uint32_t TargetManager::AllocateTargetId() {
    if (next_target_id_ > std::numeric_limits<std::uint32_t>::max()) {
        throw std::overflow_error("TargetManager exhausted uint32 target IDs");
    }
    return static_cast<std::uint32_t>(next_target_id_++);
}

void TargetManager::UpdateMatchedTarget(ManagedTarget* target, const TargetObservation& observation,
                                        const std::int64_t update_timestamp_ns) {
    const bool reactivate_processed = CooldownExpired(*target, update_timestamp_ns);
    target->raw_position = observation.position;
    target->filtered_position =
        FilteredPosition(target->filtered_position, observation.position, config_.ema_alpha);
    target->confidence = observation.confidence;
    target->last_observation_ns = observation.timestamp_ns;
    target->depth_valid = observation.depth_valid;
    target->missed_frames = 0U;

    if (target->state == TrackingState::kProcessed && !reactivate_processed) {
        return;
    }
    if (reactivate_processed) {
        target->state = TrackingState::kTentative;
        target->hit_count = 1U;
        target->processed_at_ns = 0;
        return;
    }

    ++target->hit_count;
    if (target->state == TrackingState::kLost) {
        target->state = target->hit_count >= config_.confirm_frames ? TrackingState::kConfirmed
                                                                    : TrackingState::kTentative;
    } else if (target->state == TrackingState::kTentative &&
               target->hit_count >= config_.confirm_frames) {
        target->state = TrackingState::kConfirmed;
    }
}

void TargetManager::CreateTarget(const TargetObservation& observation) {
    ManagedTarget target;
    target.target_id = AllocateTargetId();
    target.class_name = observation.class_name;
    target.confidence = observation.confidence;
    target.raw_position = observation.position;
    target.filtered_position = observation.position;
    target.first_observation_ns = observation.timestamp_ns;
    target.last_observation_ns = observation.timestamp_ns;
    target.hit_count = 1U;
    target.depth_valid = observation.depth_valid;
    target.state =
        config_.confirm_frames == 1U ? TrackingState::kConfirmed : TrackingState::kTentative;
    targets_.push_back(std::move(target));
}

void TargetManager::AgeUnmatchedTargets(const std::vector<bool>& matched_targets,
                                        const std::int64_t update_timestamp_ns) {
    for (std::size_t index = 0U; index < targets_.size(); ++index) {
        if (matched_targets[index]) {
            continue;
        }
        auto& target = targets_[index];
        ++target.missed_frames;
        if ((target.state == TrackingState::kTentative ||
             target.state == TrackingState::kConfirmed) &&
            target.missed_frames >= config_.lost_frames) {
            target.state = TrackingState::kLost;
        }
        if (target.state == TrackingState::kProcessed &&
            CooldownExpired(target, update_timestamp_ns) &&
            target.missed_frames >= config_.lost_frames) {
            target.state = TrackingState::kLost;
        }
    }
}

void TargetManager::RemoveExpiredTargets(const std::int64_t update_timestamp_ns) {
    const std::size_t lost_retirement_frames = config_.lost_frames * 2U;
    targets_.erase(std::remove_if(targets_.begin(), targets_.end(),
                                  [this, lost_retirement_frames,
                                   update_timestamp_ns](const ManagedTarget& target) {
                                      if (target.state == TrackingState::kLost) {
                                          return target.missed_frames > lost_retirement_frames;
                                      }
                                      return target.state == TrackingState::kProcessed &&
                                             CooldownExpired(target, update_timestamp_ns) &&
                                             target.missed_frames > lost_retirement_frames;
                                  }),
                   targets_.end());
}

const std::vector<ManagedTarget>& TargetManager::Update(
    const std::vector<TargetObservation>& observations, const std::int64_t update_timestamp_ns) {
    if (update_timestamp_ns < 0 ||
        (last_update_timestamp_ns_ >= 0 && update_timestamp_ns < last_update_timestamp_ns_)) {
        return targets_;
    }
    last_update_timestamp_ns_ = update_timestamp_ns;

    std::vector<TargetObservation> valid_observations;
    valid_observations.reserve(observations.size());
    for (const auto& observation : observations) {
        if (IsValidObservation(observation) && observation.timestamp_ns <= update_timestamp_ns) {
            valid_observations.push_back(observation);
        }
    }

    std::vector<AssociationCandidate> candidates;
    for (std::size_t target_index = 0U; target_index < targets_.size(); ++target_index) {
        const auto& target = targets_[target_index];
        for (std::size_t observation_index = 0U; observation_index < valid_observations.size();
             ++observation_index) {
            const auto& observation = valid_observations[observation_index];
            if (target.class_name != observation.class_name) {
                continue;
            }
            const double distance = Distance(target.filtered_position, observation.position);
            if (distance <= config_.max_match_distance) {
                candidates.push_back({distance, target_index, observation_index, target.target_id});
            }
        }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const AssociationCandidate& left, const AssociationCandidate& right) {
                  if (left.distance != right.distance) {
                      return left.distance < right.distance;
                  }
                  if (left.target_id != right.target_id) {
                      return left.target_id < right.target_id;
                  }
                  return left.observation_index < right.observation_index;
              });

    std::vector<bool> matched_targets(targets_.size(), false);
    std::vector<bool> matched_observations(valid_observations.size(), false);
    std::vector<std::size_t> accepted_observations;
    for (const auto& candidate : candidates) {
        if (matched_targets[candidate.target_index] ||
            matched_observations[candidate.observation_index]) {
            continue;
        }
        UpdateMatchedTarget(&targets_[candidate.target_index],
                            valid_observations[candidate.observation_index], update_timestamp_ns);
        matched_targets[candidate.target_index] = true;
        matched_observations[candidate.observation_index] = true;
        accepted_observations.push_back(candidate.observation_index);
    }

    AgeUnmatchedTargets(matched_targets, update_timestamp_ns);

    for (std::size_t observation_index = 0U; observation_index < valid_observations.size();
         ++observation_index) {
        if (matched_observations[observation_index]) {
            continue;
        }
        const auto& observation = valid_observations[observation_index];
        const bool duplicate = std::any_of(
            accepted_observations.begin(), accepted_observations.end(),
            [&observation, &valid_observations, this](const std::size_t accepted_index) {
                const auto& accepted = valid_observations[accepted_index];
                return observation.class_name == accepted.class_name &&
                       Distance(observation.position, accepted.position) <=
                           config_.max_match_distance;
            });
        if (duplicate) {
            continue;
        }
        CreateTarget(observation);
        accepted_observations.push_back(observation_index);
    }

    RemoveExpiredTargets(update_timestamp_ns);
    return targets_;
}

bool TargetManager::MarkProcessed(const std::uint32_t target_id, const std::int64_t timestamp_ns) {
    if (timestamp_ns < 0) {
        return false;
    }
    const auto target = std::find_if(
        targets_.begin(), targets_.end(),
        [target_id](const ManagedTarget& candidate) { return candidate.target_id == target_id; });
    if (target == targets_.end() || target->state == TrackingState::kLost ||
        timestamp_ns < target->last_observation_ns) {
        return false;
    }
    target->state = TrackingState::kProcessed;
    target->processed_at_ns = timestamp_ns;
    target->missed_frames = 0U;
    return true;
}

}  // namespace robot_perception
