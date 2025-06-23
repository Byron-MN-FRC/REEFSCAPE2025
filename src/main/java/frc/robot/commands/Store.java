// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PoseSetter;
import frc.robot.Robot;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.ElevatorS1;
import frc.robot.subsystems.ElevatorS2;
import frc.robot.subsystems.Shoulder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Store extends SequentialCommandGroup {
  /** Creates a new Store. */
  public Store(Shoulder m_shoulder, ElevatorS1 m_elevatorS1, ElevatorS2 m_elevatorS2, Coral m_claw) {
    // Add Commands here:
    // Also add parallel commands using the
    //
    addCommands(
        new MoveShoulder(m_shoulder),
        Commands.parallel(
            new MoveElevatorS1(m_elevatorS1),
            new MoveElevatorS2(m_elevatorS2)),
        new InstantCommand(() -> m_claw.coralZero()),
        new InstantCommand(() -> Robot.getInstance().currentArrangementOthers(PoseSetter.Stored)));

  }

}
