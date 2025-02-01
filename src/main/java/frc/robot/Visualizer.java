package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Visualizer extends SubsystemBase {
  @AutoLogOutput private LoggedMechanism2d m_main;
  private LoggedMechanismRoot2d m_root;

  private LoggedMechanismLigament2d m_frame;
  private LoggedMechanismLigament2d m_elevatorBase;
  private LoggedMechanismLigament2d m_elevatorStage1;
  private LoggedMechanismLigament2d m_elevatorCarriage;

  private LoggedMechanismLigament2d m_elevatorTarget;

  private Elevator m_elevator;

  public Visualizer(Elevator elevator) {
    m_elevator = elevator;

    m_main = new LoggedMechanism2d(Units.inchesToMeters(36), Units.inchesToMeters(72.0));
    m_frame = new LoggedMechanismLigament2d("drive", 30.75, 0);
    m_elevatorBase =
        new LoggedMechanismLigament2d(
            "elevator_base",
            ElevatorConstants.baseLength,
            90,
            20,
            new Color8Bit(Color.kPaleTurquoise));
    m_elevatorStage1 =
        new LoggedMechanismLigament2d(
            "elevator_stage1",
            ElevatorConstants.stage1Length,
            0,
            15,
            new Color8Bit(Color.kDarkSeaGreen));
    m_elevatorCarriage =
        new LoggedMechanismLigament2d(
            "elevator_carriage",
            ElevatorConstants.carriageLength,
            0,
            10,
            new Color8Bit(Color.kChartreuse));
    m_elevatorTarget =
        new LoggedMechanismLigament2d("elevator_target", 0, 0, 5, new Color8Bit(Color.kGreen));

    m_root = m_main.getRoot("root", (36 - 30.75) / 2.0, 1.25);
    m_root
        .append(m_frame)
        .append(m_elevatorBase)
        .append(m_elevatorStage1)
        .append(m_elevatorCarriage);

    SmartDashboard.putData("Pos", (Sendable) m_elevator.setVoltage(() -> 1));
    SmartDashboard.putData("NegPos", (Sendable) m_elevator.setVoltage(() -> -1));
  }

  public void periodic() {
    m_elevatorTarget.setLength(m_elevator.getTargetMeters());
    if (m_elevator.getPositionMeters() < ElevatorConstants.stage1Travel) {
      m_elevatorStage1.setLength(m_elevator.getPositionMeters() + ElevatorConstants.stage1Length);
    } else {
      m_elevatorStage1.setLength(ElevatorConstants.stage1Travel + ElevatorConstants.stage1Length);
      m_elevatorCarriage.setLength(m_elevator.getPositionMeters() - ElevatorConstants.stage1Travel);
    }
  }
}
