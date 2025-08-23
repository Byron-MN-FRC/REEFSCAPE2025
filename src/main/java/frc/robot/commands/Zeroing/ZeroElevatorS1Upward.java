package frc.robot.commands.Zeroing;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorS1;

/**
 *
 */
public class ZeroElevatorS1Upward extends Command {

        private final ElevatorS1 m_elevatorS1;
 
    public ZeroElevatorS1Upward(ElevatorS1 subsystem) {

        m_elevatorS1 = subsystem;
        addRequirements(m_elevatorS1);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevatorS1.setElevatorZeroingS1Upward();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_elevatorS1.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_elevatorS1.getBottomSwitch();
    }

}
