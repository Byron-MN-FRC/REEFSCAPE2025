// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PoseSetter;
import frc.robot.Robot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroAll extends SequentialCommandGroup {
  /** Creates a new Store. */
  public ZeroAll(Shoulder m_shoulder, Elevator m_elevator, Claw m_claw) {
    // Add Commands here:
    addCommands(
      new CoralClawDrop(m_claw),
      new InstantCommand(() -> m_claw.coralZero()),
      new InstantCommand(() -> m_claw.algaeZero()),
      new ZeroElevatorS1(m_elevator),
      new HomeElevatorS1(m_elevator),
      new ZeroElevatorS2(m_elevator),
      new HomeElevatorS2(m_elevator),
      new ZeroShoulder(m_shoulder),
      new HomeShoulder(m_shoulder),
      new InstantCommand(() -> Robot.getInstance().currentArrangementOthers(PoseSetter.Zero))); 

  }

}
