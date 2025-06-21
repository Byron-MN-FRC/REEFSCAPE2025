// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PoseSetter;
import frc.robot.Robot;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.ElevatorS1;
import frc.robot.subsystems.Shoulder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroAll extends SequentialCommandGroup {
  /** Creates a new Store. */
  public ZeroAll(Shoulder m_shoulder, ElevatorS1 m_elevator, Coral m_claw, Algae m_algae) {
    // Add Commands here:
    addCommands(
        // stops intake motors
        new InstantCommand(() -> m_claw.coralZero()),
        new InstantCommand(() -> m_algae.algaeZero()),

        Commands.parallel(
            Commands.sequence(
                new ZeroUpElevatorS1(m_elevator).withTimeout(666),
                new ZeroDownElevatorS1(m_elevator).withTimeout(666),
                new HomeElevatorS1(m_elevator).withTimeout(666)),
            Commands.sequence(
                new ZeroUpElevatorS2(m_elevator).withTimeout(666),
                new ZeroDownElevatorS2(m_elevator).withTimeout(666),
                new HomeElevatorS2(m_elevator).withTimeout(666)),
            Commands.sequence(
                new ZeroShoulder(m_shoulder).withTimeout(666),
                new HomeShoulder(m_shoulder).withTimeout(666)
            )
        ),

        new InstantCommand(() -> Robot.getInstance().currentArrangementOthers(PoseSetter.Zero)));
  }
}
