// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveElevatorS1;
import frc.robot.commands.MoveElevatorS2;
import frc.robot.commands.MoveShoulder;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.ElevatorS1;
import frc.robot.subsystems.ElevatorS2;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreZero extends SequentialCommandGroup {
  /** Creates a new PreZero. */
  public PreZero(Shoulder m_shoulder, ElevatorS1 m_elevatorS1, ElevatorS2 m_elevatorS2, Coral m_claw, Algae m_algae) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.parallel(
        new InstantCommand(() -> m_claw.coralZero()),
        new InstantCommand(() -> m_algae.algaeZero()),
        Commands.parallel(
          new MoveElevatorS1(m_elevatorS1),
          new MoveElevatorS2(m_elevatorS2)),
        new MoveShoulder(m_shoulder)
      )
    );
  }
}
