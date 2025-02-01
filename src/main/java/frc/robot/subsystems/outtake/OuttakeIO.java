package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public static class OuttakeIOInputs {
    public double appliedVolts = 0.0;

    public double topCurrentAmps = 0.0;
    public double topTempCelsius = 0.0;

    public double bottomCurrentAmps = 0.0;
    public double bottomTempCelsius = 0.0;
  }

  public default void processInputs(final OuttakeIOInputsAutoLogged inputs) {}

  public default void setVoltage(double voltage) {}
}
