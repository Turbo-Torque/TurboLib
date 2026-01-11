// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "networktables/StructArrayTopic.h"
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
  std::optional<photon::PhotonPipelineResult> lastResult;

  nt::StructArrayPublisher<frc::Pose2d> visionTargetPublisher;

  bool seesTag = false;

 public:
  TurboPhotonCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace, frc::AprilTagField field);
  void UpdateSim(const frc::Pose2d& robotPose);
  std::vector<structure::PoseTimestampPair> FetchPose();
  std::optional<photon::PhotonPipelineResult> GetLastResult() const { return lastResult; }
  static int GetNumTargets(const photon::PhotonPipelineResult& result) { return result.GetTargets().size(); }
  bool SeesTag() const { return seesTag; }
};
}  // namespace turbolib::perception
