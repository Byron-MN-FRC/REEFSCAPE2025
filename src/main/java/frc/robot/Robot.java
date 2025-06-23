// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private InstantCommand m_autonomousReefLevel;

  private static final RobotContainer m_robotContainer = new RobotContainer();

  private double targetSlow = 1;

  public static boolean kUseLimelight = true;
  public static boolean VISIONTEST = false;
  public static boolean COMMAND_DEBUG = false;
  public static boolean DRIVE_TO_POSITION_DEBUG = false;

  public Robot() {
    // m_robotContainer = new RobotContainer();
        LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.limelightName);
        HttpCamera frontCam = new HttpCamera("FrontCam", "http://10.48.59.11:5800");
        CameraServer.addCamera(frontCam);
        HttpCamera backCam = new HttpCamera("BackCam", "http://10.48.59.12:5800");
        CameraServer.addCamera(backCam);
  }

  public static RobotContainer getInstance(){
    return m_robotContainer;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // SmartDashboard.putBoolean("usell", kUseLimelight);

    // slow down
      if(Robot.getInstance().m_elevatorS2.stage2motor.getPosition().getValueAsDouble() >= Constants.ElevatorConstants.stage2UpperLimit - 1
        && Robot.getInstance().m_elevatorS1.stage1motor.getPosition().getValueAsDouble() >= Constants.ElevatorConstants.stage1UpperLimit - 1)
        {targetSlow = .5;}
      else if (Robot.getInstance().m_elevatorS2.stage2motor.getPosition().getValueAsDouble() >= Constants.ElevatorConstants.stage2UpperLimit - 1)
        {targetSlow = .75;}
      else targetSlow = 1;

    if (!Robot.getInstance().isSlowBtn){
    Robot.getInstance().percentSlow += (targetSlow - Robot.getInstance().percentSlow) * Constants.SwerveConstants.smoothingFactor;
    }


    
    /*
     * This example of adding Limelight is very simple and may not be sufficient for
     * on-field use.
     * Users typically need to provide a standard deviation that scales with the
     * distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible,
     * though exact implementation
     * of how to use vision should be tuned per-robot and to the team's
     * specification.
     */
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.limelightName, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.limelightName);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 1.5 && !Robot.getInstance().m_vision.tempDisable) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
            Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {
    Robot.getInstance().m_elevatorS1.stopMotor();
    Robot.getInstance().m_elevatorS2.stopMotor();
    Robot.getInstance().m_shoulder.stopShoulder();
    Robot.getInstance().m_coral.coralZero();
    Robot.getInstance().m_algae.algaeZero();

    // CHARACTERIZING
    // SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    kUseLimelight = true;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousReefLevel = m_robotContainer.getSelectedAutoLevel();

    if (m_autonomousCommand != null) {
      
      if (m_autonomousReefLevel != null) {
        m_autonomousReefLevel.schedule();
      } else {
        new InstantCommand(() -> Constants.Selector.PlacementSelector.setCurrentRow(3));
      }

      m_autonomousCommand.schedule();
      
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
  kUseLimelight = true;
  Robot.getInstance().m_vision.tempDisable = false;
  }
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.limelightName);
    }

    // SignalLogger.start();
    // TURN ON FOR CHARACTERIZING

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
