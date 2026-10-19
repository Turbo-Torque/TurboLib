// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <string>
#include <vector>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Transform3d.h"
#include "networktables/DoubleArrayTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "turbolib/structure/PoseTimestampPair.hpp"

namespace turbolib::perception {
// ADDED: Limelight counterpart to TurboPhotonCamera — reads the Limelight's own on-device pose
// estimate (botpose_wpiblue) instead of running PhotonVision. The AprilTag field layout is still
// configured on the Limelight itself (its web UI) — that part isn't settable from robot code.
// The camera-to-robot mounting offset (cameraInBotSpace) IS settable from code, unlike the field
// layout: this publishes it to the Limelight's "camerapose_robotspace_set" NT entry, which the
// device reads and uses in place of (overrides) whatever's set in its web UI. Per Limelight's own
// docs, set the web UI camera pose to all zeros once so this programmatic value is the only source
// of truth — otherwise it's ambiguous which one the device actually uses.
// Uses the single blue-origin field convention (botpose_wpiblue) for both alliances, matching how
// the rest of this codebase already treats field positions (e.g. FieldConstants::kRedHubPose).
// No simulation support — unlike PhotonVision, there's no vendor-provided Limelight simulator.
class TurboLimelightCamera {
private:
  std::string limelightName;
  nt::DoubleArraySubscriber botposeSubscriber;
  nt::DoubleSubscriber targetValidSubscriber;
  nt::DoubleArrayPublisher cameraPosePublisher;

  bool seesTag = false;

public:
  TurboLimelightCamera(const std::string& limelightName, const frc::Transform3d& cameraInBotSpace);

  std::vector<structure::PoseTimestampPair> FetchPose();

  bool SeesTag() const { return seesTag; }
};
}  // namespace turbolib::perception
