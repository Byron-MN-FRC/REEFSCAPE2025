
package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class ZeroShoulderSecond extends Command {

    private final Shoulder m_shoulder;

    public ZeroShoulderSecond(Shoulder subsystem) {

        m_shoulder = subsystem;
        addRequirements(m_shoulder);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shoulder.setShoulderZeroingSecond();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shoulder.stopShoulder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_shoulder.isShoulderTripped();
    }

}