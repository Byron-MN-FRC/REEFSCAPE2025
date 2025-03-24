// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonResetRotation extends Command {

    private final Vision m_Vision;
    private final CommandSwerveDrivetrain drivetrain;

    /** Creates a new AutonResetRotation. */
    public AutonResetRotation(CommandSwerveDrivetrain drivetrain, Vision m_Vision) {
        this.drivetrain = drivetrain;
        this.m_Vision = m_Vision;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Vision.tempDisable(0.5);
        drivetrain.resetRotation(drivetrain.getOperatorForwardDirection().plus(new Rotation2d(Math.PI)));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_Vision.tempDisable;
    }
}