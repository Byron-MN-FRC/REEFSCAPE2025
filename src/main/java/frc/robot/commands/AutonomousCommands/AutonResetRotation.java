// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonResetRotation extends Command {

    private final Vision m_Vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final Timer m_timer = new Timer();

    /** Creates a new AutonResetRotation. */
    public AutonResetRotation(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        m_Vision = vision;
        m_timer.reset();
        addRequirements(m_Vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        /* This uses reset rotation rather than seed field centric
           equivalent to calling seedFieldCentric and adding Math.PI */
        drivetrain.resetRotation(drivetrain.getOperatorForwardDirection().plus(new Rotation2d(Math.PI)));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(0.5);
    }
}