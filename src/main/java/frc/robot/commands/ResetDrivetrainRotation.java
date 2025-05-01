package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class ResetDrivetrainRotation extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Timer m_timer = new Timer();

    public ResetDrivetrainRotation(CommandSwerveDrivetrain drivetrain, Vision m_vision) {
        this.m_drivetrain = drivetrain;
        addRequirements(m_vision);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_drivetrain.seedFieldCentric();

    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(0.5);
    }
}