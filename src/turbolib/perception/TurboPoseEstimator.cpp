// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "turbolib/perception/TurboPoseEstimator.hpp"

#include <cassert>
#include <vector>

#include "frc/geometry/Pose2d.h"
#include "telemetrykit/core/AlertManager.h"
#include "turbolib/structure/PoseTimestampPair.hpp"

using namespace turbolib::perception;

frc::Pose2d TurboPoseEstimator::GetPose2D() {
  return poseEstimator.GetEstimatedPosition();
}

void TurboPoseEstimator::ResetEstimatorPosition(const frc::Rotation2d& gyroAngle,
                                                const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                                                const frc::Pose2d& pose) {
  poseEstimator.ResetPosition(gyroAngle, modulePositions, pose);
}

void TurboPoseEstimator::UpdateWithOdometryAndVision(const frc::Rotation2d& gyroAngle,
                                                     const std::array<frc::SwerveModulePosition, 4>& modulePositions) {
  poseEstimator.Update(gyroAngle, modulePositions);
  UpdateWithAllAvailableVisionMeasurements();
}

void TurboPoseEstimator::TryVisionUpdateWithCamera(turbolib::perception::TurboPhotonCamera& camera) {
  const std::vector<turbolib::structure::PoseTimestampPair> visionPoses = camera.FetchPose();

  for (const auto& pair : visionPoses) {
    poseEstimator.AddVisionMeasurement(pair.getPose(), pair.getLatency());
  }
}

void TurboPoseEstimator::UpdateWithAllAvailableVisionMeasurements() {
  if (localizationCameras.empty()) {
    auto& alertManager = tkit::AlertManager::GetInstance();
    alertManager.Warning("no_localization_cameras", "No localization cameras to update.");
    return;
  }

  tkit::AlertManager::GetInstance().ClearManualAlert("no_localization_cameras");

  for (auto& camera : localizationCameras) {
    TryVisionUpdateWithCamera(*camera);
  }
}

void TurboPoseEstimator::UpdateAllSims(frc::Pose2d pose) {
  for (auto& camera : localizationCameras) {
    camera->UpdateSim(pose);
  }
}

bool TurboPoseEstimator::SeesTag() const {
  bool seesTag = false;

  for (const auto& camera : localizationCameras) {
    if (camera->SeesTag()) {
      seesTag = true;
      break;
    }
  }

  return seesTag;
}
