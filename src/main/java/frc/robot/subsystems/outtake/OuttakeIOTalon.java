package frc.robot.subsystems.outtake;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.RobotMap;

public class OuttakeIOTalon implements OuttakeIO {
  private final TalonSRX topTalon;
  private final TalonSRX bottomTalon;

  public OuttakeIOTalon() {
    topTalon = new TalonSRX(RobotMap.Outtake.top);
    bottomTalon = new TalonSRX(RobotMap.Outtake.bottom);

    topTalon.configPeakCurrentLimit(30);
    topTalon.setNeutralMode(NeutralMode.Coast);

    bottomTalon.follow(topTalon);
    bottomTalon.setInverted(InvertType.OpposeMaster);
  }

  @Override
  public void processInputs(OuttakeIOInputsAutoLogged inputs) {
    inputs.appliedVolts = topTalon.getMotorOutputVoltage();

    inputs.topCurrentAmps = topTalon.getSupplyCurrent();
    inputs.topTempCelsius = topTalon.getTemperature();
    inputs.bottomCurrentAmps = bottomTalon.getSupplyCurrent();
    inputs.bottomTempCelsius = bottomTalon.getTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    topTalon.set(TalonSRXControlMode.PercentOutput, voltage / 12.0);
  }
}
