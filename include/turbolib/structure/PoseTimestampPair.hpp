// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <array>
#include <utility>
#include "frc/geometry/Pose2d.h"
#include "units/time.h"

namespace turbolib::structure {
class PoseTimestampPair {
  frc::Pose2d pose;
  units::second_t latency;
  std::array<double, 3> stdDevs;

 public:
  PoseTimestampPair(const frc::Pose2d pose, const units::second_t latency, const std::array<double, 3>&& stdDevs)
      : pose(pose), latency(latency), stdDevs(std::move(stdDevs)) {}

  [[nodiscard]]
  frc::Pose2d getPose() const {
    return pose;
  }

  [[nodiscard]]
  units::second_t getLatency() const {
    return latency;
  }

  [[nodiscard]]
  std::array<double, 3> getStdDevs() const {
    return stdDevs;
  }
};
}  // namespace turbolib::structure
