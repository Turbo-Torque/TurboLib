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
  void SetCameraInBotSpace(const frc::Transform3d& cameraInBotSpace);
};
}
