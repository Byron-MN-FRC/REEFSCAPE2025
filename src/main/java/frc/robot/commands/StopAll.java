package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;

public class StopAll extends Command {
    Elevator m_elevator;
    Shoulder m_shoulder;
    Coral m_coral;
    Algae m_algae;

    public StopAll(Elevator m_elevator, Shoulder m_shoulder, Coral m_coral, Algae m_algae) {

        this.m_elevator = m_elevator;
        this.m_shoulder = m_shoulder;
        this.m_coral = m_coral;
        this.m_algae = m_algae;

        addRequirements(m_elevator, m_shoulder, m_coral, m_algae);
    }

    @Override
    public void initialize() {
        m_elevator.stopBothMotors();
        m_shoulder.stopShoulder();
        m_algae.algaeZero();
        m_coral.coralZero();

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
