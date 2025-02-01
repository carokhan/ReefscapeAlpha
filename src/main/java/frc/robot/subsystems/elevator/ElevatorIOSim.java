package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.SimConstants;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim stage1Sim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          ElevatorConstants.gearing,
          ElevatorConstants.stage1Mass,
          ElevatorConstants.drumRadius,
          ElevatorConstants.stage1Offset,
          ElevatorConstants.stage1Travel,
          true,
          ElevatorConstants.stage1Offset);
  private final ElevatorSim carriageSim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          ElevatorConstants.gearing,
          ElevatorConstants.carriageMass,
          ElevatorConstants.drumRadius,
          ElevatorConstants.stage1Offset + ElevatorConstants.carriageOffset,
          ElevatorConstants.carriageTravel,
          true,
          ElevatorConstants.stage1Offset + ElevatorConstants.carriageOffset);
  private double volts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    stage1Sim.update(SimConstants.loopTime);
    carriageSim.update(SimConstants.loopTime);

    inputs.positionMeters =
        stage1Sim.getPositionMeters()
            + ElevatorConstants.carriageOffset
            + carriageSim.getPositionMeters();
    inputs.velocityMetersPerSecond =
        Math.max(stage1Sim.getVelocityMetersPerSecond(), carriageSim.getVelocityMetersPerSecond());
    inputs.appliedVolts = volts;
    inputs.currentAmps = Math.max(stage1Sim.getCurrentDrawAmps(), carriageSim.getCurrentDrawAmps());
    inputs.leftTempCelsius = 20.0;
    inputs.rightTempCelsius = 20.0;
  }

  @Override
  public void setVoltage(double voltage) {
    volts = voltage;
    if ((stage1Sim.getPositionMeters()
            + ElevatorConstants.carriageOffset
            + carriageSim.getPositionMeters())
        > ElevatorConstants.travelOrder[0]) {
      carriageSim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    } else {
      stage1Sim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }
  }

  @Override
  public void resetEncoder(double position) {}
}
