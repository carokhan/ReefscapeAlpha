package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  private ElevatorFeedforward carriageFeedforward =
      new ElevatorFeedforward(
          ElevatorConstants.Gains.kSCarriageFeedforward,
          ElevatorConstants.Gains.kGCarriageFeedforward,
          ElevatorConstants.Gains.kVCarriageFeedforward,
          ElevatorConstants.Gains.kACarriageFeedforward);
  private ElevatorFeedforward stage1CarriageFeedforward =
      new ElevatorFeedforward(
          ElevatorConstants.Gains.kSStage1CarriageFeedforward,
              ElevatorConstants.Gains.kGStage1CarriageFeedforward,
          ElevatorConstants.Gains.kVStage1CarriageFeedforward,
              ElevatorConstants.Gains.kAStage1CarriageFeedforward);
  private ProfiledPIDController pid =
      new ProfiledPIDController(
          ElevatorConstants.Gains.kP,
          ElevatorConstants.Gains.kI,
          ElevatorConstants.Gains.kD,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration));

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  public double currentFilterValue = 0.0;

  public boolean hasZeroed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    currentFilterValue = currentFilter.calculate(inputs.currentAmps);
  }

  public Command setTarget(DoubleSupplier meters) {
    return this.run(
        () -> {
          double ff =
              (inputs.positionMeters < ElevatorConstants.travelOrder[0])
                  ? stage1CarriageFeedforward.calculateWithVelocities(
                      inputs.velocityMetersPerSecond, pid.getSetpoint().velocity)
                  : carriageFeedforward.calculateWithVelocities(
                      inputs.velocityMetersPerSecond, pid.getSetpoint().velocity);
          double volts = pid.calculate(inputs.positionMeters, meters.getAsDouble()) + ff;

          io.setVoltage(volts);
          inputs.targetPositionMeters = meters.getAsDouble();
        });
  }

  public Command setTarget(double meters) {
    return this.setTarget(() -> meters);
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
        () -> {
          io.setVoltage(volts.getAsDouble());
        });
  }

  public Command setVoltage(double voltage) {
    return this.setVoltage(() -> voltage);
  }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              io.setVoltage(-0.5);
              Logger.recordOutput("Elevator/Setpoint", Double.NaN);
            })
        .until(() -> currentFilterValue > 20.0)
        .finallyDo(
            (interrupted) -> {
              if (!interrupted) {
                io.resetEncoder(0.0);
                hasZeroed = true;
              }
            });
  }

  public double getPositionMeters() {
    return inputs.positionMeters + ElevatorConstants.visualizerOffset;
  }

  public double getTargetMeters() {
    return inputs.targetPositionMeters + ElevatorConstants.visualizerOffset;
  }
}
