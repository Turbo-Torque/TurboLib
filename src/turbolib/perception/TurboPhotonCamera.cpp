// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "turbolib/perception/TurboPhotonCamera.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "turbolib/structure/PoseTimestampPair.hpp"

using namespace turbolib::perception;

TurboPhotonCamera::TurboPhotonCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace,
                                     frc::AprilTagField field, bool enableSim)
    : layout(frc::AprilTagFieldLayout::LoadField(field)), camera(cameraName), poseEstimator(layout, cameraInBotSpace) {
  visionPosePublisher =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("DriveSubsystem/VisionPose").Publish();

  if (enableSim) {
    auto cameraProp = photon::SimCameraProperties();
    cameraProp.SetCalibration(640, 480, 75_deg);
    cameraProp.SetCalibError(0.25, 0.08);
    cameraProp.SetFPS(units::hertz_t{20});
    cameraProp.SetAvgLatency(20_ms);
    cameraProp.SetLatencyStdDev(3_ms);

    systemSim.emplace("main");
    cameraSim.emplace(photon::PhotonCameraSim(&camera, cameraProp));

    cameraSim->EnableRawStream(true);
    cameraSim->EnabledProcessedStream(true);
    cameraSim->EnableDrawWireframe(false);

    systemSim->AddAprilTags(layout);
    systemSim->AddCamera(&cameraSim.value(), cameraInBotSpace);

    frc::SmartDashboard::PutData("Sim Field", &systemSim->GetDebugField());
  }
}

void TurboPhotonCamera::UpdateSim(const frc::Pose2d& robotPose) {
  if (systemSim.has_value()) {
    systemSim->Update(robotPose);
  }
}

std::vector<turbolib::structure::PoseTimestampPair> TurboPhotonCamera::FetchPose() {
  std::vector<turbolib::structure::PoseTimestampPair> poses;

  for (const auto& result : camera.GetAllUnreadResults()) {
    lastResult = result;
    if (auto visionEst = poseEstimator.EstimateCoprocMultiTagPose(result)) {
      poses.emplace_back(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp);
    } else {
      auto singleTargetEst = poseEstimator.EstimateLowestAmbiguityPose(result);
      if (singleTargetEst) {
        poses.emplace_back(singleTargetEst->estimatedPose.ToPose2d(), singleTargetEst->timestamp);
      }
    }
  } 

  if (poses.empty()) {
    seesTag = false;
  } else {
    seesTag = true;

    visionPosePublisher.Set(poses.front().getPose());
  }

  return poses;
}
