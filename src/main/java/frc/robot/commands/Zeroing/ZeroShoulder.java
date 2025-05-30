
package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class ZeroShoulder extends Command {

    private final Shoulder m_shoulder;

    public ZeroShoulder(Shoulder subsystem) {

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
        m_shoulder.setShoulderZeroing();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_shoulder.isShoulderTripped();
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}