// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PoseSetter;
import frc.robot.Robot;
import frc.robot.commands.MoveElevatorS1;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ElevatorS1;
import frc.robot.subsystems.ElevatorS2;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroAllNew extends SequentialCommandGroup {
  /** Creates a new Store. */
  public ZeroAllNew(Shoulder m_shoulder, ElevatorS1 m_elevatorS1, ElevatorS2 m_elevatorS2, Coral m_claw, Algae m_algae) {
    // Add Commands here:
    addCommands(

      //TODO add timeouts
      new InstantCommand(() -> m_claw.coralZero()),
      new InstantCommand(() -> m_algae.algaeZero()),
      Commands.sequence(
        new ZeroElevatorS2Upward(m_elevatorS2),
        new ZeroElevatorS2Downward(m_elevatorS2),
        new HomeElevatorS2(m_elevatorS2)),
      Commands.parallel(
        Commands.sequence(
          new ZeroShoulderFirst(m_shoulder),
          new ZeroShoulderSecond(m_shoulder),
          new HomeShoulder(m_shoulder)),
        Commands.sequence(
          new ZeroElevatorS1Downward(m_elevatorS1),
          new ZeroElevatorS1Upward(m_elevatorS1),
          new HomeElevatorS1(m_elevatorS1))),
          
      new InstantCommand(() -> Robot.getInstance().goalArrangementOthers(PoseSetter.Zero)),
      new InstantCommand(() -> Robot.getInstance().currentArrangementOthers(PoseSetter.Zero))
      );

  }

}
