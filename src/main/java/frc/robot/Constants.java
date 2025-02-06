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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    public static final double mass = DriveConstants.mass;
    public static final double moi = DriveConstants.angularMOI;
    public static final double bumperFront = Units.inchesToMeters(33.5 / 2);
    public static final double bumperBack = Units.inchesToMeters(33.5 / 2);
    public static final double bumperSide = Units.inchesToMeters(32.5 / 2);
    public static final double frontModX = DriveConstants.trackWidthY / 2;
    public static final double frontLeftY = DriveConstants.trackWidthX / 2;
    public static final double backModX = -DriveConstants.trackWidthY / 2;
    public static final double backLeftY = -DriveConstants.trackWidthX / 2;
    public static final double wheelRadius = DriveConstants.wheelRadius;
    public static final double motorRevWheelRev = DriveConstants.driveRatio;
    public static final double motorMaxSpeed =
        5880 * 0.8 * Math.PI * 2 / 60; // RPM (free speed) * 0.8 * 2pi/60 =
    // radPerSec (loaded speed)
    public static final double motorMaxTorque =
        3.28 / 181 * DriveConstants.driveCurrent; // (kT * currentLimit)
  }

  public static class DriveConstants {
    public static final boolean wheelsStraight = false;

    public static final double trackWidthX = Units.inchesToMeters(19.75);
    public static final double trackWidthY = Units.inchesToMeters(20.75);
    public static final double trackBaseRadius = Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);

    public static final double wheelRadius = Units.inchesToMeters(2);

    public static final double mass = Units.lbsToKilograms(56);

    public static final double driveRatio = 5.36;
    public static final double driveMOI = 0.025;
    public static final double turnRatio = 150.0 / 7.0;
    public static final double turnMOI = 0.004;

    public static final double driveConversion = (driveRatio) * (1.0 / (wheelRadius * 2 * Math.PI));
    public static final double turnConversion = 2 * Math.PI / turnRatio;
    public static final double turnVelocityConversion = turnConversion / 60;

    public static final int driveCurrent = 40; // 70
    public static final int turnCurrent = 40; // 30

    public static final double odometeryFrequency = 250;
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

  public static class VisionConstants {
    public static final boolean useVision = true;

    public static class CameraInfo {

      public String cameraName;
      public String model;
      public Transform3d robotToCamera;
      public Rotation2d diagFOV;
      public int[] cameraRes;

      public CameraInfo(
          String cameraName,
          String model,
          Transform3d robotToCamera,
          Rotation2d diagFOV,
          int[] cameraRes) {
        this.cameraName = cameraName;
        this.model = model;
        this.robotToCamera = robotToCamera;
        this.diagFOV = diagFOV;
        this.cameraRes = cameraRes;
      }
    }

    public static CameraInfo leftCamera =
        new CameraInfo(
            "LeftCamera",
            "Spinel OV9281-1086-B",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(7.875),
                    Units.inchesToMeters(10.25),
                    Units.inchesToMeters(8.25)),
                new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                    .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)))),
            Rotation2d.fromDegrees(95),
            new int[] {1280, 720});

    public static CameraInfo rightCamera =
        new CameraInfo(
            "RightCamera",
            "Arducam OV2311-1086-A",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(7.875),
                    Units.inchesToMeters(-10.25),
                    Units.inchesToMeters(8.25)),
                new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                    .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-30.0)))),
            Rotation2d.fromDegrees(75),
            new int[] {1600, 1200});

    public static final Matrix<N3, N1> singleTagStdDev =
        VecBuilder.fill(0.8, 0.8, Double.MAX_VALUE);
    public static final Matrix<N3, N1> multiTagStdDev = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }
}
