// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double trackWidth = Units.inchesToMeters(18.75);
  public static final double wheelBase = Units.inchesToMeters(18.75);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final double wheelRadius = Units.inchesToMeters(2);
  public static final double mass = Units.lbsToKilograms(103.726);

  public static final double driveRatio = (50.0 * 16.0 * 45.0) / (16.0 * 28.0 * 15.0);
  public static final double turnRatio = 150.0 / 7.0;

  public static final double drivePositionConversion =
      2 * Math.PI / driveRatio; // Rotor Rotations -> Wheel
  // Radians
  public static final double driveVelocityConversion =
      (2 * Math.PI) / 60.0 / driveRatio; // Rotor RPM -> Wheel
  // Rad/Sec
  public static final double turnPositionConversion =
      2 * Math.PI / turnRatio; // Rotations -> Radians
  public static final double turnVelocityConversion =
      turnPositionConversion / 60.0; // RPM -> Rad/Sec

  public static final DCMotor driveGearbox = DCMotor.getNEO(1);
  public static final int driveCurrent = 50;

  public static final DCMotor turnGearbox = DCMotor.getNEO(1);
  public static final int turnCurrent = 20;
  public static final boolean turnInverted = true;

  public static final boolean turnEncoderInverted = false;

  public static final double maxLinearVelocity = Units.feetToMeters(20.4);
  public static final double maxLinearAccel = Units.feetToMeters(driveBaseRadius);

  public static final double maxAngularVelocity = maxLinearVelocity / driveBaseRadius;
  public static final double maxAngularAccel = 22.863;

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  public static class Gains {
    public static double kPDriveReal = 2.0; // 1.4866E-05
    public static double kDDriveReal = 0.2;
    public static double kSDriveReal = 0.17236; // 0.097715, 0.027736, 0.021057, 0.093808
    public static double kVDriveReal = (0.10324 + 0.11273 + 0.10143 + 0.10776) / 4;
    public static double kADriveReal = (0.0056151 + 0.0040328 + 0.0077964 + 0.0060387) / 4;
    public static double kPDriveSim = 2.0;
    public static double kDDriveSim = 0.2;
    public static double kSDriveSim = 0.4;
    public static double kVDriveSim = 1.93;
    public static double kADriveSim = 0.25;

    public static double kPTurnReal = 1.5; // 1.5?
    public static double kDTurnReal = 0.0;
    public static double kPTurnSim = 2.5;
    public static double kDTurnSim = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    public static final double kALinear = (0.10222 + 0.19369 + 0.18158 + 0.11887) / 4;
    public static final double kAAngular = (0.23623 + 0.23717 + 0.23954 + 0.22678) / 4;
  }

  public static final double moi = mass * trackWidth / 2 * Gains.kAAngular / Gains.kALinear; // 20.8

  public static final double odometryFrequency = 100.0;

  // PathPlanner configuration
  public static final double wheelCOF = 0.899; // Colsons
  public static final RobotConfig ppConfig =
      new RobotConfig(
          mass,
          moi,
          new ModuleConfig(
              wheelRadius,
              maxLinearVelocity,
              wheelCOF,
              driveGearbox.withReduction(driveRatio),
              driveCurrent,
              1),
          moduleTranslations);
}
