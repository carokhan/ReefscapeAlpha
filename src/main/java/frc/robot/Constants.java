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

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class RobotMap {
    public static class Drive {
      public static final int pigeon = 9;

      public static final int frontLeftDrive = 1;
      public static final int backLeftDrive = 5;
      public static final int frontRightDrive = 3;
      public static final int backRightDrive = 7;

      public static final int frontLeftTurn = 2;
      public static final int backLeftTurn = 6;
      public static final int frontRightTurn = 4;
      public static final int backRightTurn = 8;

      public static final int frontLeftEncoder = 1;
      public static final int backLeftEncoder = 2;
      public static final int frontRightEncoder = 0;
      public static final int backRightEncoder = 3;

      public static final Rotation2d frontLeftOffset = new Rotation2d(0.0);
      public static final Rotation2d frontRightOffset = new Rotation2d(0.0);
      public static final Rotation2d backLeftOffset = new Rotation2d(0.0);
      public static final Rotation2d backRightOffset = new Rotation2d(0.0);
    }

    public static class Elevator {
      public static final int left = 30;
      public static final int right = 31;
    }

    public static class Outtake {
      public static final int top = 21;
      public static final int bottom = 22;
    }
  }

  public static class AutoConstants {
    public static final double kPTranslation = 8.5;
    public static final double kDTranslation = 0.0;

    public static final double kPRotation = 3.0;
    public static final double kDRotation = 0.0;

    public static final double bumperFront = Units.inchesToMeters(33.5 / 2);
    public static final double bumperBack = Units.inchesToMeters(33.5 / 2);
    public static final double bumperSide = Units.inchesToMeters(32.5 / 2);
    public static final double frontModX = DriveConstants.trackWidthY / 2;
    public static final double frontLeftY = DriveConstants.trackWidthX / 2;
    public static final double backModX = -DriveConstants.trackWidthY / 2;
    public static final double backLeftY = -DriveConstants.trackWidthX / 2;
    public static final double motorRevWheelRev = DriveConstants.driveRatio;
    public static final double motorMaxSpeed =
        5880 * 0.8 * Math.PI * 2 / 60; // RPM (free speed) * 0.8 * 2pi/60 =
    // radPerSec (loaded speed)
    public static final double motorMaxTorque =
        3.28 / 181 * DriveConstants.driveCurrent; // (kT * currentLimit)

    public static final RobotConfig ppConfig =
        new RobotConfig(
            DriveConstants.mass,
            DriveConstants.moi,
            new ModuleConfig(
                DriveConstants.wheelRadius,
                DriveConstants.maxLinearVelocity,
                DriveConstants.wheelCOF,
                DriveConstants.driveGearbox.withReduction(DriveConstants.driveRatio),
                DriveConstants.driveCurrent,
                1),
            DriveConstants.moduleTranslations);
  }

  public static class DriveConstants {
    public static final boolean wheelsStraight = false;

    public static final double trackWidthX = Units.inchesToMeters(19.75);
    public static final double trackWidthY = Units.inchesToMeters(20.75);
    public static final double trackBaseRadius = Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    public static final Translation2d[] moduleTranslations = {
      new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)
    };

    public static final double wheelRadius = Units.inchesToMeters(2);
    public static final double wheelCOF = 0.9;

    public static final double mass = Units.lbsToKilograms(110);
    public static final double moi = 2.42042257;

    public static final double driveRatio = 5.36;
    public static final double driveMOI = 0.025;
    public static final double turnRatio = 150.0 / 7.0;
    public static final double turnMOI = 0.004;

    public static final double driveConversion = 2 * Math.PI / driveRatio;
    public static final double driveVelocityConversion = (2 * Math.PI) / 60.0 / driveRatio;
    public static final double turnConversion = 2 * Math.PI / turnRatio;
    public static final double turnVelocityConversion = turnConversion / 60;
    public static final boolean turnInverted = false;

    public static final int driveCurrent = 40; // 70
    public static final int turnCurrent = 40; // 30

    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    public static final double odometryFrequency = 100.0;
    public static final double updateFrequency = 100;

    public static final double maxLinearVelocity = Units.feetToMeters(20.4);
    // public static final double maxLinearVelocity = Units.feetToMeters(1.4);
    public static final double maxLinearAccel = 8.0;

    public static final double maxAngularVelocity = 20;
    public static final double maxAngularAccel = 10;

    public static double kPDriveReal = 2.0; // 1.4866E-05
    public static double kDDriveReal = 0.2;
    public static double kSDriveReal = 0.17236; // 0.097715, 0.027736, 0.021057, 0.093808
    public static double kVDriveReal = (0.10324 + 0.11273 + 0.10143 + 0.10776) / 4;
    public static double kADriveReal = (0.0056151 + 0.0040328 + 0.0077964 + 0.0060387) / 4;

    public static double kPTurnReal = 1.5; // 1.5?
    public static double kDTurnReal = 0.0;
    public static double turnPIDMinInput = 0; // Radians
    public static double turnPIDMaxInput = 2 * Math.PI; // Radians

    public static double kPDriveSim = 2.0;
    public static double kDDriveSim = 0.2;
    public static double kSDriveSim = 0.4;
    public static double kVDriveSim = 1.93;
    public static double kADriveSim = 0.25;

    public static double kPTurnSim = 2.5;
    public static double kDTurnSim = 0.0;

    public static double kPDriveReplay = 0.0;
    public static double kDDriveReplay = 0.0;
    public static double kSDriveReplay = 0.0;
    public static double kVDriveReplay = 0.0;
    public static double kADriveReplay = 0.0;

    public static double kPTurnReplay = 0.0;
    public static double kDTurnReplay = 0.0;

    public static final double kALinear = (0.10222 + 0.19369 + 0.18158 + 0.11887) / 4;
    public static final double kAAngular = (0.23623 + 0.23717 + 0.23954 + 0.22678) / 4;
    public static final double angularMOI = mass * trackWidthY / 2 * kAAngular / kALinear; // 20.8
  }

  public static class ControlConstants {
    public static final double deadband = 0.09375;
  }

  public static class SimConstants {
    public static final double loopTime = 0.02;
  }
}
