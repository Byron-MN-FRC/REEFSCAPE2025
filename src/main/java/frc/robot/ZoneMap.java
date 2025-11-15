package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZoneMap {
    
    public Rectangle2d rect = new Rectangle2d(new Translation2d(5, 0), new Translation2d(6.0001, Constants.VisionConstants.fieldWidth));
    public Rectangle2d topWall = new Rectangle2d(new Translation2d(-1, Constants.VisionConstants.fieldWidth), new Translation2d(Constants.VisionConstants.fieldLength+1, Constants.VisionConstants.fieldWidth+1));
    public Rectangle2d blueWall = new Rectangle2d(new Translation2d(-1, -1), new Translation2d(0, Constants.VisionConstants.fieldWidth+1));
    public Rectangle2d redWall = new Rectangle2d(new Translation2d(Constants.VisionConstants.fieldLength, -1), new Translation2d(Constants.VisionConstants.fieldLength+1, Constants.VisionConstants.fieldWidth+1));
    public Rectangle2d bottomWall = new Rectangle2d(new Translation2d(-1, -1), new Translation2d(Constants.VisionConstants.fieldLength+1, 0));
    public static ArrayList<Rectangle2d> rectList = new ArrayList<>();
    
    public ZoneMap() {
        rectList.add(topWall);
        rectList.add(blueWall);
        rectList.add(redWall);
        rectList.add(bottomWall);
    }
    
    public double getDistance(Pose2d robotPose) {
        return rect.getDistance(robotPose.getTranslation());
    }

    public double getDistance(ArrayList<Rectangle2d> list) {
        double dist = 4859;
        for ( Rectangle2d rect : list) {
            if (rect.getDistance(Robot.getInstance().drivetrain.getState().Pose.getTranslation()) < dist){
                dist = rect.getDistance(Robot.getInstance().drivetrain.getState().Pose.getTranslation());
            }
        }
        return dist;
    }

    public void test() {
         
        // SmartDashboard.putNumber("Distance is ", getDistance(new Pose2d(4.5,5,new Rotation2d()))); 
        SmartDashboard.putNumber("Distance is ", getDistance(rectList)); 

    }

}
