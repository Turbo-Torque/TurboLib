// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "turbolib/perception/TurboPoseEstimator.hpp"

#include <cassert>
#include <vector>

#include "frc/geometry/Pose2d.h"
#include "turbolib/structure/PoseTimestampPair.hpp"


using namespace turbolib::perception;

frc::Pose2d TurboPoseEstimator::GetPose2D() {
  return poseEstimator.GetEstimatedPosition();
}

void TurboPoseEstimator::ResetEstimatorPosition(const frc::Rotation2d& gyroAngle,
                                                const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                                                const frc::Pose2d& pose) {
  poseEstimator.ResetPosition(gyroAngle, modulePositions, pose);

  for (const auto& camera : localizationCameras) {
    camera->ResetHeading(gyroAngle);
  }
}

void TurboPoseEstimator::UpdateWithOdometryAndVision(const frc::Rotation2d& gyroAngle,
                                                     const std::array<frc::SwerveModulePosition, 4>& modulePositions) {
  poseEstimator.Update(gyroAngle, modulePositions);

  if (visionEnabled) {
    UpdateWithAllAvailableVisionMeasurements(gyroAngle);
  }
}

void TurboPoseEstimator::TryVisionUpdateWithCamera(turbolib::perception::TurboPhotonCamera& camera,
                                                   const frc::Rotation2d& gyroAngle) {
  const std::vector<turbolib::structure::PoseTimestampPair> visionPoses = camera.FetchPose();

  for (const auto& pair : visionPoses) {
    poseEstimator.AddVisionMeasurement(pair.getPose(), pair.getLatency());
  }
}

void TurboPoseEstimator::UpdateWithAllAvailableVisionMeasurements(const frc::Rotation2d& gyroAngle) {
  // FIXED: this used to return early (skipping the limelightCameras loop below entirely) whenever
  // localizationCameras was empty — true whenever PhotonVision cameras are disabled/commented out,
  // even with Limelight cameras configured and working. Looping over an empty vector is already a
  // no-op, so the guard wasn't needed in the first place.
  for (auto& camera : localizationCameras) {
    TryVisionUpdateWithCamera(*camera, gyroAngle);
  }

  for (auto& camera : limelightCameras) {
    for (const auto& pair : camera->FetchPose()) {
      poseEstimator.AddVisionMeasurement(pair.getPose(), pair.getLatency());
    }
  }
}



void TurboPoseEstimator::UpdateAllSims(frc::Pose2d pose) {
  for (auto& camera : localizationCameras) {
    camera->UpdateSim(pose);
  }
}

bool TurboPoseEstimator::SeesTag() const {
  return std::any_of(localizationCameras.begin(), localizationCameras.end(),
                     [](const std::unique_ptr<TurboPhotonCamera>& cam) { return cam->SeesTag(); });
}
