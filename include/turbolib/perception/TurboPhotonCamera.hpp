// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "frc/RobotBase.h"
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/simulation/PhotonCameraSim.h"
#include "photon/simulation/VisionSystemSim.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "turbolib/structure/PoseTimestampPair.hpp"

namespace turbolib::perception {
class TurboPhotonCamera {
 private:
  frc::AprilTagFieldLayout layout;
  photon::PhotonCamera camera;
  photon::PhotonPoseEstimator poseEstimator;

  std::optional<photon::VisionSystemSim> systemSim = std::nullopt;
  std::optional<photon::PhotonCameraSim> cameraSim = std::nullopt;

  bool seesTag = false;

 public:
  TurboPhotonCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace, frc::AprilTagField field,
                    bool enableSim = frc::RobotBase::IsSimulation());

  void UpdateSim(const frc::Pose2d& robotPose);
  void UpdateHeading(const frc::Rotation2d& gyroAngle);
  void ResetHeading(const frc::Rotation2d& gyroAngle);

  std::vector<structure::PoseTimestampPair> FetchPose();
  structure::PoseTimestampPair CalculateStdDevs(const photon::EstimatedRobotPose& estimatedPose,
                                                const photon::PhotonPipelineResult& result) {
    int numTags = estimatedPose.targetsUsed.size();
    double avgDistance = 0.0;

    for (const auto& target : estimatedPose.targetsUsed) {
      avgDistance += target.GetBestCameraToTarget().Translation().Norm().value();
    }

    if (numTags > 0)
      avgDistance /= numTags;

    double xyStdDev = 0.1;
    double degStdDev = 0.05;

    if (numTags == 1) {
      // Singletag scales quadratically with distance
      xyStdDev = 0.05 * std::pow(avgDistance, 2.0);
      degStdDev = 0.1 * std::pow(avgDistance, 2.0);
    } else {
      // Multitag scales linearly and drops with more tags
      xyStdDev = (0.02 * avgDistance) / numTags;
      degStdDev = (0.03 * avgDistance) / numTags;
    }

    // if more than 4.5 meters away, basically ignore
    if (avgDistance > 4.5) {
      xyStdDev = 999.0;
      degStdDev = 999.0;
    }

    return {estimatedPose.estimatedPose.ToPose2d(), estimatedPose.timestamp, {xyStdDev, xyStdDev, degStdDev}};
  }

  bool SeesTag() const { return seesTag; }
  bool IsSimulated() const { return cameraSim.has_value(); }
};
}  // namespace turbolib::perception
