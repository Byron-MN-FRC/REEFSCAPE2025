// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceAlgae extends SequentialCommandGroup {
  /** Creates a new RemoveAlgae. */
  public PlaceAlgae(Shoulder m_shoulder, Elevator m_elevator, Algae m_algae) {

    addCommands(
     Commands.parallel( 
      new MoveShoulder(m_shoulder),
      new MoveElevator(m_elevator), 
      new InstantCommand(() -> Robot.getInstance().currentArrangementPlacing())
     )
    );
  }
}
