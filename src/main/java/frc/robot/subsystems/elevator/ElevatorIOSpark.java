package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.RobotMap;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax leftSpark = new SparkMax(RobotMap.Elevator.left, MotorType.kBrushless);
  private final SparkMax rightSpark = new SparkMax(RobotMap.Elevator.right, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftSpark.getEncoder();

  private final Debouncer leftConnectedDebounce = new Debouncer(0.5);
  private final Debouncer rightConnectedDebounce = new Debouncer(0.5);

  public ElevatorIOSpark() {
    var leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.current)
        .voltageCompensation(12.0)
        .inverted(false);
    leftConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.positionConversionFactor / 60.0)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    leftConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 50.0))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    var rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.current)
        .voltageCompensation(12.0)
        .follow(RobotMap.Elevator.left)
        .inverted(true);
    rightConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.positionConversionFactor / 60.0)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    rightConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 50.0))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        leftSpark,
        5,
        () ->
            leftSpark.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    sparkStickyFault = false;
    ifOk(leftSpark, leftEncoder::getPosition, (value) -> inputs.positionMeters = value);
    ifOk(leftSpark, leftEncoder::getVelocity, (value) -> inputs.velocityMetersPerSecond = value);
    ifOk(
        leftSpark,
        new DoubleSupplier[] {leftSpark::getAppliedOutput, leftSpark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(leftSpark, leftSpark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.leftConnected = leftConnectedDebounce.calculate(!sparkStickyFault);

    ifOk(leftSpark, leftSpark::getMotorTemperature, (value) -> inputs.leftTempCelsius = value);

    sparkStickyFault = false;
    ifOk(rightSpark, rightSpark::getMotorTemperature, (value) -> inputs.rightTempCelsius = value);
    inputs.rightConnected = rightConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(double voltage) {
    leftSpark.setVoltage(voltage);
  }

  @Override
  public void resetEncoder(double position) {
    tryUntilOk(leftSpark, 5, () -> leftEncoder.setPosition(0.0));
  }
}
