package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;

public class Visualizer extends SubsystemBase {
    @AutoLogOutput
    private Mechanism2d m_main;

    private MechanismLigament2d m_elevatorStage1;
    private MechanismLigament2d m_elevatorStage2;

    private MechanismLigament2d m_elevatorStage2Target;

    private MechanismRoot2d m_elevatorStage1Root;
    private MechanismRoot2d m_elevatorStage2Root;

    private Elevator m_elevator;

    public Visualizer(Elevator elevator) {
        m_elevator = elevator;

        m_main = new Mechanism2d(Units.inchesToMeters(36), Units.inchesToMeters(72.0));
        m_elevatorStage1Root = m_main.getRoot("elevator_stage1_base", Units.inchesToMeters(2),
                Units.inchesToMeters(39.0));
        m_elevatorStage2Root = m_main.getRoot("elevator_stage2_base", Units.inchesToMeters(2),
                Units.inchesToMeters(16.25));
        m_main.getRoot("robot", Units.inchesToMeters(30.75), Units.inchesToMeters(7)).append(
                new MechanismLigament2d("frane", Units.inchesToMeters(24), 0, 2, new Color8Bit(Color.kLightCoral)));

        m_elevatorStage1.append(new MechanismLigament2d("Stage 1", 0, 0));
    }
}
