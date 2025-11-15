// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "turbolib/perception/TurboPhotonCamera.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "frc/RobotBase.h"
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/simulation/PhotonCameraSim.h"
#include "photon/simulation/SimCameraProperties.h"
#include "turbolib/structure/PoseTimestampPair.hpp"
#include "units/frequency.h"
#include "units/time.h"

using namespace turbolib::perception;

TurboPhotonCamera::TurboPhotonCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace,
                                     frc::AprilTagField field)
    : layout(frc::AprilTagFieldLayout::LoadField(field)),
      camera(cameraName),
      poseEstimator(layout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, cameraInBotSpace) {
  if constexpr (frc::RobotBase::IsSimulation()) {
    auto cameraProp = photon::SimCameraProperties();
    cameraProp.SetCalibration(1280, 720, 75_deg);
    cameraProp.SetCalibError(0.25, 0.08);
    cameraProp.SetFPS(units::hertz_t{30});
    cameraProp.SetAvgLatency(35_ms);
    cameraProp.SetLatencyStdDev(5_ms);

    systemSim.emplace("main");
    cameraSim.emplace(photon::PhotonCameraSim(&camera, cameraProp));

    cameraSim->EnableRawStream(true);
    cameraSim->EnabledProcessedStream(true);
    cameraSim->EnableDrawWireframe(true);

    systemSim->AddAprilTags(layout);
    systemSim->AddCamera(&cameraSim.value(), cameraInBotSpace);

    frc::SmartDashboard::PutData("Sim Field", &systemSim->GetDebugField());
  }

  visionTargetPublisher =
      nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::Pose2d>(cameraName + "/targets").Publish();
}

void TurboPhotonCamera::UpdateSim(const frc::Pose2d& robotPose) {
  if constexpr (frc::RobotBase::IsSimulation()) {
    systemSim->Update(robotPose);
  }
}

std::vector<turbolib::structure::PoseTimestampPair> TurboPhotonCamera::FetchPose() {
  std::vector<turbolib::structure::PoseTimestampPair> poses;

  for (const auto& result : camera.GetAllUnreadResults()) {
    lastResult = result;
    if (auto visionEst = poseEstimator.Update(result)) {
      poses.emplace_back(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp);
    }
  }

  if (poses.empty()) {
    seesTag = false;
  } else {
    seesTag = true;
  }

  return poses;
}
