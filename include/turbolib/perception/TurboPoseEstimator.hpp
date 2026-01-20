// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "frc/apriltag/AprilTagFields.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "turbolib/perception/TurboPhotonCamera.hpp"

namespace turbolib::perception {
class TurboPoseEstimator {
 private:
  frc::SwerveDrivePoseEstimator<4> poseEstimator;

  std::vector<turbolib::perception::TurboPhotonCamera> localizationCameras;

 public:
  TurboPoseEstimator(const frc::Rotation2d& gyroAngle, const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                     const frc::Pose2d& initialPose, frc::SwerveDriveKinematics<4> kinematics)
      : poseEstimator(kinematics, gyroAngle, modulePositions, initialPose) {}

  frc::Pose2d GetPose2D();
  void ResetEstimatorPosition(const frc::Rotation2d& gyroAngle,
                              const std::array<frc::SwerveModulePosition, 4>& modulePositions, const frc::Pose2d& pose);
  void UpdateWithOdometryAndVision(const frc::Rotation2d& gyroAngle,
                                   const std::array<frc::SwerveModulePosition, 4>& modulePositions);
  void TryVisionUpdateWithCamera(turbolib::perception::TurboPhotonCamera& camera);
  void UpdateWithAllAvailableVisionMeasurements();

  void AddLocalizationCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace,
                             frc::AprilTagField field) {
    localizationCameras.emplace_back(cameraName, cameraInBotSpace, field);
  }

  std::optional<photon::PhotonPipelineResult> GetLastVisionResult(int cameraIdx) const {
    if (cameraIdx < 0 || static_cast<size_t>(cameraIdx) >= localizationCameras.size()) {
      return std::nullopt;
    }

    return localizationCameras[cameraIdx].GetLastResult();
  }

  bool SeesTag() const;
};
}  // namespace turbolib::perception
