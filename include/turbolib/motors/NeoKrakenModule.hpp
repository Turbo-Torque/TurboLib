// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <rev/SparkMax.h>

#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "ctre/phoenix6/CANBus.hpp"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "units/current.h"
#include "units/length.h"
#include "units/velocity.h"

inline constexpr double GEAR_RATIO = 6.75;
inline constexpr units::inch_t WHEEL_DIAMETER = 4_in;

namespace turbolib::motors {
class NeoKrakenModule final {
  std::string name;

  ctre::phoenix6::CANBus canBus;
  ctre::phoenix6::hardware::TalonFX driveMotor;
  rev::spark::SparkMax steerMotor;
  ctre::phoenix6::hardware::CANcoder encoderObject;

  double setpoint{};

  frc::SimpleMotorFeedforward<units::meters> ff;
  frc::PIDController driveController, steerController;

  constexpr static double kVelocityMultiplier = (1 / GEAR_RATIO / 60) * (units::meter_t{WHEEL_DIAMETER}.value() * M_PI);
  constexpr static double kPositionMultiplier = (1 / GEAR_RATIO) * (units::meter_t{WHEEL_DIAMETER}.value() * M_PI);
  constexpr static double kCanCoderMultiplier = 2 * M_PI;

 public:
  NeoKrakenModule(const std::string& name, int driveID, int steerID, int encoderID, const std::string& can = "");

  void ConfigPIDInternal();
  static void ConfigDriveMotor(ctre::phoenix6::hardware::TalonFX& target);
  static void ConfigSteerMotor(rev::spark::SparkMax& target);
  static void SetupEncoder(ctre::phoenix6::hardware::CANcoder& encoder);
  static void CurrentLimitsDrive(ctre::phoenix6::configs::TalonFXConfiguration& config);
  void SetModuleState(frc::SwerveModuleState state);
  double GetEncoderPosition();
  double GetPosition();
  frc::SwerveModuleState GetModuleState();
  frc::SwerveModulePosition GetModulePosition();
  units::meters_per_second_t GetVelocity();
  units::ampere_t GetCurrentDraw();
};
}  // namespace turbolib::motors
