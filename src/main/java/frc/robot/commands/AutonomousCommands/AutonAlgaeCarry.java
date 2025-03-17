// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonAlgaeCarry extends Command {
  private final Claw m_claw;
  /** Creates a new AutonAlgaeCarry. */
  public AutonAlgaeCarry(Claw subsystem) {
    
    m_claw = subsystem;
    addRequirements(m_claw);
  }
// run this with a race group or deadline group in pathplanner
  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
