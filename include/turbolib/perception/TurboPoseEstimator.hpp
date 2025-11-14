// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <array>

#include "frc/apriltag/AprilTagFields.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "networktables/StructTopic.h"
#include "turbolib/constants/Constants.hpp"
#include "turbolib/perception/TurboPhotonCamera.hpp"

namespace turbolib::perception {
class TurboPoseEstimator {
 private:
  frc::SwerveDrivePoseEstimator<4> poseEstimator;

  std::array<turbolib::perception::TurboPhotonCamera, 2> localizationCameras = {
      {{CameraConstants::kLocalizationCamOneName, CameraConstants::kLocalizationCamOneOffset,
        frc::AprilTagField::k2025ReefscapeAndyMark},
       {CameraConstants::kLocalizationCamTwoName, CameraConstants::kLocalizationCamTwoOffset,
        frc::AprilTagField::k2025ReefscapeAndyMark}}};

  nt::StructSubscriber<frc::Pose2d> simPoseTopic =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Pose").Subscribe(frc::Pose2d{});

 public:
  TurboPoseEstimator(const frc::Rotation2d& gyroAngle, const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                     const frc::Pose2d& initialPose)
      : poseEstimator(DriveSubsystemConstants::kKinematics, gyroAngle, modulePositions, initialPose) {}

  frc::Pose2d GetPose2D() const;
  void ResetEstimatorPosition(const frc::Rotation2d& gyroAngle,
                              const std::array<frc::SwerveModulePosition, 4>& modulePositions, const frc::Pose2d& pose);
  void UpdateWithOdometryAndVision(const frc::Rotation2d& gyroAngle,
                                   const std::array<frc::SwerveModulePosition, 4>& modulePositions);
  void TryVisionUpdateWithCamera(turbolib::perception::TurboPhotonCamera& camera);
  void UpdateWithAllAvailableVisionMeasurements();

  bool SeesTag() const;
};
}  // namespace turbolib::perception
