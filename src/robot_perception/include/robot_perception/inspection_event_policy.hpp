#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "robot_perception/target_manager.hpp"

namespace robot_perception {

struct InspectionEventPolicyConfig {
  bool enabled = true;
  std::vector<std::string> allowed_classes{"chair"};
  double min_confidence = 0.5;
};

class InspectionEventPolicy {
 public:
  explicit InspectionEventPolicy(InspectionEventPolicyConfig config = {});

  bool ShouldEmit(const ManagedTarget& target);
  bool HasEmitted(std::uint32_t target_id) const;

 private:
  InspectionEventPolicyConfig config_;
  std::unordered_set<std::uint32_t> emitted_target_ids_;
};

}  // namespace robot_perception
