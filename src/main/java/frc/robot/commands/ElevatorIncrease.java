// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ElevatorS2;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorIncrease extends Command {

  private final ElevatorS2 m_elevatorS2;

  public ElevatorIncrease(ElevatorS2 subsystem) {

    m_elevatorS2 = subsystem;
    addRequirements(m_elevatorS2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Robot.COMMAND_DEBUG) System.out.println("elevator increase");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorS2.increase();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
