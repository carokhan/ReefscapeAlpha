package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double targetPositionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public boolean limitSwitch = false;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public boolean leftConnected = false;
    public double leftTempCelsius = 0.0;

    public boolean rightConnected = false;
    public double rightTempCelsius = 0.0;
  }

  public default void updateInputs(final ElevatorIOInputsAutoLogged inputs) {}

  public default void setVoltage(final double voltage) {}

  public default void stop() {
    setVoltage(0);
  }

  public default void resetEncoder(final double position) {}

  public default void resetEncoder() {
    resetEncoder(0.0);
  }
}
