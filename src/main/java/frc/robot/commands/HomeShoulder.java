
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PoseSetter;
import frc.robot.Robot;
import frc.robot.subsystems.Shoulder;

public class HomeShoulder extends Command {

    private final Shoulder m_shoulder;

    public HomeShoulder(Shoulder subsystem) {

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
        m_shoulder.setShoulderHoming();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shoulder.setShoulderMotorPosition(Constants.ShoulderConstants.shoulderSensorLimit);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_shoulder.isShoulderTripped();
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}