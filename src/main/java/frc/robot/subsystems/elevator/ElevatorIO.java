package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public boolean limitSwitch = false;
  }

  public void updateInputs(final ElevatorIOInputsAutoLogged inputs);

  public void setVoltage(final double voltage);

  public default void stop() {
    setVoltage(0);
  }

  public void resetEncoder(final double position);

  public default void resetEncoder() {
    resetEncoder(0.0);
  }
}
