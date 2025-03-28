package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

    private static final Vision m_Vision = new Vision();
    public boolean tempDisable = false;
    public double timestampToReEnable;
    private Pose2d autoStartPose = new Pose2d();
    public int lastTargetFront = 1;
    public int lastTargetBack = 1;

    public static Vision getInstance() {
        return m_Vision;
    }

    public Vision() {
        LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.limelightName, -0.09, 0, 0.44, 0, 0,0);
        LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.limelightName2, -0.093, 0, 0.44, 0, 21, 0);
    }

    @Override
    public void periodic() {
        
        updateTargetData(Constants.VisionConstants.limelightName);
        updateTargetData(Constants.VisionConstants.limelightName2);
        
        // SmartDashboard.putNumber("lastTargetFront", lastTargetFront);
        // SmartDashboard.putNumber("lastTargetBack", lastTargetBack);
      
        if (timestampToReEnable < Utils.getCurrentTimeSeconds() &&tempDisable  == true){
            tempDisable = false; 
        }

        // SmartDashboard.putBoolean("tempDisable", tempDisable);

        // SmartDashboard.putString("placementPosition: " , autoStartPose.toString());
        // SmartDashboard.putString("currentPosition: ", Robot.getInstance().drivetrain.getState().Pose.toString());

        if (DriverStation.isAutonomous() && !DriverStation.isEnabled()) {
            // For auto set-up
            if (!autoStartPose.equals(new Pose2d())) {
                Translation2d currentT2D = Robot.getInstance().drivetrain.getState().Pose.getTranslation();
                double distance = autoStartPose.getTranslation().getDistance(currentT2D);
                // difference between the goal angle and current angle arccos(cos(a-b))
                double rot_distance = Math.acos(autoStartPose.getRotation().getCos() * 
                     Robot.getInstance().drivetrain.getState().Pose.getRotation().getCos() + 
                     autoStartPose.getRotation().getSin() * 
                     Robot.getInstance().drivetrain.getState().Pose.getRotation().getSin());

                // SmartDashboard.putNumber("Auto config distance", distance);
                // SmartDashboard.putNumber("Auto config rotation distance", rot_distance);
                if (distance < 0.2 && (Units.radiansToDegrees(rot_distance) < 4)) {

                    LimelightHelpers.setLEDMode_ForceOn(Constants.VisionConstants.limelightName);
                } else {
                    LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.limelightName);
                }
            } else {
                LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.limelightName);
            }
        }
    }
    
    public Alliance MyAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red ? Alliance.Red : Alliance.Blue;
        } else {
            return null;
        }
    }

    public void updateTargetData(String llName) {
        if (LimelightHelpers.getTV(llName)) {
            int fidID = (int) LimelightHelpers.getFiducialID(llName);
            if (!(fidID >= 1) || !(fidID <= 22)) {
                    fidID = 1;
            }
            if (llName.equals(Constants.VisionConstants.limelightName)) {
                lastTargetFront = fidID;
            }
            if (llName.equals(Constants.VisionConstants.limelightName2)) {
                lastTargetBack = fidID;
            }
        }
    }
    
    /**
     *  Temporarily disables the addVisionMeasurements method in Robot.java
     *  
     *  The purpose of this method is to remove errors caused during resetting
     *  the rotation of the robot when the cameras can see an April Tag
     * 
     *  @Param seconds The time period to disable for (tested at .5 seconds)
     *  @return void
     */
    public void tempDisable(double seconds) {
        tempDisable = true;
        double currentTime = Utils.getCurrentTimeSeconds();
        timestampToReEnable = currentTime + seconds;
    }

    public void updateAutoStartPosition(String autoName) {

        // Instant Command is the name of the "None" Auto

        if (!autoName.equals("InstantCommand")) {
            try {
                autoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingDifferentialPose(); 
            } catch (Exception e){
                System.out.println(e.getMessage());
                autoStartPose = new Pose2d();
            }
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                autoStartPose = FlippingUtil.flipFieldPose(autoStartPose);
            }
        } else {
            autoStartPose = new Pose2d();
        } 

    }
}