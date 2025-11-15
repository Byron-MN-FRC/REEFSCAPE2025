// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ZoneMap;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithForcefield extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController joystick;
  double percentSlow;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  // 3/4 of a rotation per second max angular velocity

  /** Creates a new DriveWithForcefield. */
  public DriveWithForcefield(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, double percentSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    this.percentSlow = percentSlow;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double distance = Robot.getInstance().zoneMap.getDistance(drivetrain.getState().Pose);
    double distance = Robot.getInstance().zoneMap.getDistance(ZoneMap.rectList);
    double extraSlownessFactor = 1;
    if (distance < 1.0) {
      extraSlownessFactor = distance;
      if (extraSlownessFactor < 0.09) {
        extraSlownessFactor = 0.09;
      }

    }
    drivetrain.setControl(Robot.getInstance().drive
        .withVelocityX(-joystick.getLeftY() * MaxSpeed * percentSlow * extraSlownessFactor)
        // Drive forward with negative Y (forward)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed * percentSlow * extraSlownessFactor)
        // Drive left with negative X (left)
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * percentSlow)
      // Drive counterclockwise with negative X (left)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // private void mixSpeeds(DriveState state, ZoneMap map) {

  // }
}
