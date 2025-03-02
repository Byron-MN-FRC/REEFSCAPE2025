// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

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

  private boolean kUseLimelight = true;

  public Robot() {
    // m_robotContainer = new RobotContainer();
        // HttpCamera frontCam = new HttpCamera("FrontCam", "http://10.48.59.11:5800");
        // CameraServer.addCamera(frontCam);
        // HttpCamera backCam = new HttpCamera("BackCam", "http://10.48.59.12:5800");
        // CameraServer.addCamera(backCam);
  }

  public static RobotContainer getInstance(){
    return m_robotContainer;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Stage 2", m_robotContainer.getTopStage2());
    SmartDashboard.putBoolean("Coral", m_robotContainer.getCoralDetect());
    SmartDashboard.putBoolean("ShoulderTripped", m_robotContainer.getShoulderTripped());
    
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

      LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.limeLightName, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.limeLightName);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
            Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }

      // //keep if testing the two limelights independently
      LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.limeLightName2, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.limeLightName2);
      if (llMeasurement2 != null && llMeasurement2.tagCount > 0 && omegaRps < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement2.pose,
            Utils.fpgaToCurrentTime(llMeasurement2.timestampSeconds));
      }

      // // Track error between the two cameras
      if (llMeasurement != null && llMeasurement2 != null){
//        double error = Math.abs(llMeasurement.pose.getX()-llMeasurement2.pose.getX()) +
//          Math.abs(llMeasurement.pose.getY()-llMeasurement2.pose.getY());
//        SmartDashboard.putNumber("Error between cams", error);
        SmartDashboard.putString("Error between cams", String.format("x1: %f x2: %f y1: %f y2: %f tag: %d %d", 
          llMeasurement.pose.getX(), llMeasurement2.pose.getX(),
          llMeasurement.pose.getY(), llMeasurement2.pose.getY(), llMeasurement.tagCount, llMeasurement2.tagCount));
      }
    }

    SmartDashboard.putNumber("tagselected", Robot.getInstance().globalCurrNumSelected); 
  }

  @Override
  public void disabledInit() {
    Robot.getInstance().m_elevator.stopBothMotors();
    Robot.getInstance().m_shoulder.stopShoulder();
    Robot.getInstance().m_claw.zero();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    kUseLimelight = false;
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
  }
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // RUN TO ZERO PLEASE //

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
