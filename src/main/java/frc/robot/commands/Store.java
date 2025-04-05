// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PoseSetter;
import frc.robot.Robot;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Store extends SequentialCommandGroup {
  /** Creates a new Store. */
  public Store(Shoulder m_shoulder, Elevator m_elevator, Coral m_claw){
    // Add Commands here:
    // Also add parallel commands using the
    //
    addCommands(
          new MoveShoulder(m_shoulder),
          new MoveElevator(m_elevator),
          new InstantCommand(() -> m_claw.coralZero()),
        new InstantCommand(() -> Robot.getInstance().currentArrangementOthers(PoseSetter.Stored)));

  }

}
