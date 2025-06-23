package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.ElevatorS1;
import frc.robot.subsystems.ElevatorS2;
import frc.robot.subsystems.Shoulder;

public class StopAll extends Command {
    ElevatorS1 m_elevatorS1;
    ElevatorS2 m_elevatorS2;
    Shoulder m_shoulder;
    Coral m_coral;
    Algae m_algae;

    public StopAll(ElevatorS1 m_elevatorS1, ElevatorS2 m_elevatorS2, Shoulder m_shoulder, Coral m_coral, Algae m_algae) {

        this.m_elevatorS1 = m_elevatorS1;
        this.m_elevatorS2 = m_elevatorS2;
        this.m_shoulder = m_shoulder;
        this.m_coral = m_coral;
        this.m_algae = m_algae;

        addRequirements(m_elevatorS1, m_elevatorS2, m_shoulder, m_coral, m_algae);
    }

    @Override
    public void initialize() {
        m_elevatorS1.stopMotor();
        m_elevatorS2.stopMotor();
        m_shoulder.stopShoulder();
        m_algae.algaeZero();
        m_coral.coralZero();

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
