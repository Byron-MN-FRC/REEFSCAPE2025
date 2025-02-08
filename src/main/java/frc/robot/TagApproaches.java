package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TagApproach.gameTarget;

public class TagApproaches {
    public AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private TagApproach[] tagArray;

    private static TagApproaches _TagApproaches = new TagApproaches();

    public static TagApproaches getInstance() {
        return _TagApproaches;
    }

    // distance in meters from robot center to front edge of bumper.
    public final double robotWidth = 0.5;

    private double rw = 0.451; // Robot circumference in meters
    Pose2d pose;

    public TagApproaches() {
        tagArray = new TagApproach[23];

        double poseOffsetx = robotWidth * Math.cos(getTagAngle(1));
        double poseOffsety = robotWidth * Math.sin(getTagAngle(1));
        pose = calcNewPose(1, poseOffsetx, poseOffsety, 0);
        tagArray[0] = new TagApproach(1, Alliance.Red, gameTarget.CoralStation, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(2));
        poseOffsety = robotWidth * Math.sin(getTagAngle(2));
        pose = calcNewPose(1, poseOffsetx, poseOffsety, 0);
        tagArray[1] = new TagApproach(2, Alliance.Red, gameTarget.CoralStation, pose);

        pose = calcNewPose(3, 0, -robotWidth, 0);
        tagArray[2] = new TagApproach(3, Alliance.Red, gameTarget.Processor, pose);

        pose = calcNewPose(4, robotWidth, 0, 0);
        tagArray[3] = new TagApproach(4, Alliance.Blue, gameTarget.Barge, pose);

        pose = calcNewPose(5, robotWidth, 0, 0);
        tagArray[4] = new TagApproach(5, Alliance.Red, gameTarget.Barge, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(6));
        poseOffsety = robotWidth * Math.sin(getTagAngle(6));
        pose = calcNewPose(6, poseOffsetx, poseOffsety, 0);
        tagArray[5] = new TagApproach(6, Alliance.Red, gameTarget.Reef, pose);

        pose = calcNewPose(7, robotWidth, 0, 0);
        tagArray[6] = new TagApproach(7, Alliance.Red, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(8));
        poseOffsety = robotWidth * Math.sin(getTagAngle(8));
        pose = calcNewPose(8, poseOffsetx, poseOffsety, 0);
        tagArray[7] = new TagApproach(8, Alliance.Red, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(9));
        poseOffsety = robotWidth * Math.sin(getTagAngle(9));
        pose = calcNewPose(9, poseOffsetx, poseOffsety, 0);
        tagArray[8] = new TagApproach(9, Alliance.Red, gameTarget.Reef, pose);
        
        pose = calcNewPose(10, -robotWidth, 0, 0);
        tagArray[9] = new TagApproach(10, Alliance.Red, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(11));
        poseOffsety = robotWidth * Math.sin(getTagAngle(11));
        pose = calcNewPose(11, poseOffsetx, poseOffsety, 0);
        tagArray[10] = new TagApproach(11, Alliance.Red, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(12));
        poseOffsety = robotWidth * Math.sin(getTagAngle(12));
        pose = calcNewPose(12, poseOffsetx, poseOffsety, 0);
        tagArray[11] = new TagApproach(12, Alliance.Blue, gameTarget.CoralStation, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(13));
        poseOffsety = robotWidth * Math.sin(getTagAngle(13));
        pose = calcNewPose(13, poseOffsetx, poseOffsety, 0);
        tagArray[12] = new TagApproach(13, Alliance.Blue, gameTarget.CoralStation, pose);

        pose = calcNewPose(14, -robotWidth, 0, 0);
        tagArray[13] = new TagApproach(14, Alliance.Blue, gameTarget.Barge, pose);

        pose = calcNewPose(15, -robotWidth, 0, 0);
        tagArray[14] = new TagApproach(15, Alliance.Red, gameTarget.Barge, pose);

        pose = calcNewPose(16, 0, robotWidth, 0);
        tagArray[15] = new TagApproach(16, Alliance.Blue, gameTarget.Processor, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(17));
        poseOffsety = robotWidth * Math.sin(getTagAngle(17));
        pose = calcNewPose(17, poseOffsetx, poseOffsety, 0);
        tagArray[16] = new TagApproach(17, Alliance.Blue, gameTarget.Reef, pose);

        pose = calcNewPose(18, -robotWidth, 0, 0);
        tagArray[17] = new TagApproach(18, Alliance.Blue, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(19));
        poseOffsety = robotWidth * Math.sin(getTagAngle(19));
        pose = calcNewPose(19, poseOffsetx, poseOffsety, 0);
        tagArray[18] = new TagApproach(19, Alliance.Blue, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(20));
        poseOffsety = robotWidth * Math.sin(getTagAngle(20));
        pose = calcNewPose(20, poseOffsetx, poseOffsety, 0);
        tagArray[19] = new TagApproach(20, Alliance.Blue, gameTarget.Reef, pose);

        pose = calcNewPose(21, robotWidth, 0, 0);
        tagArray[20] = new TagApproach(21, Alliance.Blue, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(22));
        poseOffsety = robotWidth * Math.sin(getTagAngle(22));
        pose = calcNewPose(22, poseOffsetx, poseOffsety, 0);
        tagArray[21] = new TagApproach(22, Alliance.Blue, gameTarget.Reef, pose);

        // numbers past this point are not tags, but rather user specifified positions

        pose = calcNewPose(4, 7, 0);
        tagArray[22] = new TagApproach("testPose1", Alliance.Red, gameTarget.Reef, pose);
    }

    private Pose2d calcNewPose(int id, double arbX, double arbY, double arbAngle) {
        Pose2d tagPose = FieldLayout.getTagPose(id).get().toPose2d();

        // return new Pose2d(tagPose.getX() + arbX,
        //         tagPose.getY() + arbY,
        //         new Rotation2d(Math.toRadians(arbAngle)));
        return new Pose2d(tagPose.getX() + arbX,
                tagPose.getY() + arbY,
                new Rotation2d(tagPose.getRotation().getRadians() + Math.toRadians(arbAngle) + Math.toRadians(180)));
    }

    private Pose2d calcNewPose(double arbX, double arbY, double arbAngle) {
        return new Pose2d(arbX, arbY, new Rotation2d(Math.toRadians(arbAngle)));
    }

    public int FiduciaryNumber(int tagID) {
        return tagArray[tagID - 1].FiduciaryNumber();
    }

    public Alliance TagAlliance(int tagID) {
        return tagArray[tagID - 1].TagAlliance();
    }

    public gameTarget GameTarget(int tagID) {
        return tagArray[tagID - 1].GameTarget();
    }

    public Pose2d DesiredRobotPos(int tagID) {
        int indexInArray = tagID - 1;
        // Alliance alliance = Robot.getInstance().m_Vision.MyAlliance();
        // if (indexInArray > 21 && alliance != null && alliance != tagArray[indexInArray].TagAlliance()) {
            
        //    return RotatePose2d(indexInArray);
                    
        // }
        
        Pose2d goalPose = tagArray[indexInArray].DesiredPos();

        if (tagArray[indexInArray].GameTarget() == gameTarget.Reef){ 
            System.out.println("shifting");
            return shiftReefAllign(goalPose, Robot.getInstance().GLOBALOFFSET);
        }
        return goalPose;
                // return tagArray[indexInArray].DesiredPos();
        // return new Pose2d();
    }

    public Pose2d TagFieldPose2d(int tagID) {
        return FieldLayout.getTagPose(tagID).get().toPose2d();
    }

    public Pose2d RotatePose2d(int arrayIndex) {
        Pose2d oppOrigin = new Pose2d(FieldLayout.getFieldLength(), FieldLayout.getFieldWidth(), new Rotation2d(Math.PI));
        return tagArray[arrayIndex].DesiredPos().relativeTo(oppOrigin);
    }

    public double getTagAngle(int id) {
        return FieldLayout.getTagPose(id).get().getRotation().toRotation2d().getDegrees();
    }

    public Pose2d shiftReefAllign(Pose2d goalBeforeShift, double robotRelativeShift) {
        System.out.println(robotRelativeShift);
        Rotation2d goalAngle = goalBeforeShift.getRotation();
        Translation2d oldTranslation = goalBeforeShift.getTranslation();
        Translation2d offsetTranslation = new Translation2d(robotRelativeShift, goalAngle.plus(Rotation2d.fromDegrees(90)));
        Translation2d newGoalTranslation = oldTranslation.plus(offsetTranslation);
        return new Pose2d(newGoalTranslation, goalAngle);
    }

}