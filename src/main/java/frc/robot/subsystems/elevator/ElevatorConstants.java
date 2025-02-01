package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final double stageOverlap = Units.inchesToMeters(3.5 * 2.0);

  public static final double baseLength = Units.inchesToMeters(35.5);
  public static final double stage1Offset = Units.inchesToMeters(6.25);
  public static final double stage1Length = Units.inchesToMeters(32.25);
  public static final double carriageOffset = Units.inchesToMeters(1);
  public static final double carriageLength = Units.inchesToMeters(9);

  public static final double stage1Travel = baseLength - stage1Offset - stageOverlap;
  public static final double carriageTravel = stage1Length - carriageOffset + 1 - carriageLength;
  public static final double[] travelOrder = {stage1Travel, carriageTravel};

  public static final double carriageMass = Units.lbsToKilograms(19.464);
  public static final double stage1Mass = Units.lbsToKilograms(5.239) + carriageMass;

  public static double drumRadius = 5.0 / 1000.0 * 30 / (2.0 * Math.PI);
  public static final double gearing = (3.0 / 1.0) * (52.0 / 26.0);
  public static final double positionConversionFactor = drumRadius * 2 * Math.PI / gearing;

  public static final int current = 40;

  public static class Gains {
    public static final double kSCarriageFeedforward = 0.0;
    public static final double kGCarriageFeedforward = 0.0;
    public static final double kVCarriageFeedforward = 0.0;
    public static final double kACarriageFeedforward = 0.0;

    public static final double kSStage1CarriageFeedforward = 0.0;
    public static final double kGStage1CarriageFeedforward = 0.0;
    public static final double kVStage1CarriageFeedforward = 0.0;
    public static final double kAStage1CarriageFeedforward = 0.0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static final double maxVelocity = 0.0;
  public static final double maxAcceleration = 0.0;

  public static final double visualizerOffset = 0.0;
}
