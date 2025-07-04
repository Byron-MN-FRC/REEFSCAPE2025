
package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;

public class ZeroShoulderFirst extends Command {

    private final Shoulder m_shoulder;

    public ZeroShoulderFirst(Shoulder subsystem) {

        m_shoulder = subsystem;
        addRequirements(m_shoulder);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shoulder.setCurrentLimit(Constants.ShoulderConstants.shoulderCalibrationCurrentLimit);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shoulder.setShoulderZeroing();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shoulder.stopShoulder();
        m_shoulder.setCurrentLimit(Constants.ShoulderConstants.shoulderStandardCurrentLimit);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_shoulder.isShoulderTripped() || m_shoulder.shoulderCombinedStallHandler.isStalled();
    }

}