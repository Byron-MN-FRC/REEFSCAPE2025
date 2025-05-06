package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class ResetRotation extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    Vision m_vision;
    Timer m_timer = new Timer();

    public ResetRotation(CommandSwerveDrivetrain drivetrain, Vision vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_timer.restart();
    }

    @Override
    public void initialize() {
        m_vision.tempDisable = true;
        m_drivetrain.seedFieldCentric();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_vision.tempDisable = false;
    }
}
