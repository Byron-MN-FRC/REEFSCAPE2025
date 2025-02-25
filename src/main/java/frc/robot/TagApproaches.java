package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TagApproach.gameTarget;

public class TagApproaches {
    public AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private TagApproach[] tagArray;

    private static TagApproaches _TagApproaches = new TagApproaches();

    public static TagApproaches getInstance() {
        return _TagApproaches;
    }

    // distance in meters from robot center to front edge of bumper.
    public final double robotWidth = 0.5;
    Pose2d pose;
    double poseOffsetx;
    double poseOffsety;

    public TagApproaches() {
        tagArray = new TagApproach[22];

        poseOffsetx = robotWidth * Math.cos(getTagAngle(1));
        poseOffsety = robotWidth * Math.sin(getTagAngle(1));
        pose = calcNewPose(1, poseOffsetx, poseOffsety, 0);
        tagArray[0] = new TagApproach(1, Alliance.Red, gameTarget.CoralStation, pose);
        
        poseOffsetx = robotWidth * Math.cos(getTagAngle(2));
        poseOffsety = robotWidth * Math.sin(getTagAngle(2));
        pose = calcNewPose(2, poseOffsetx, poseOffsety, 0);
        tagArray[1] = new TagApproach(2, Alliance.Red, gameTarget.CoralStation, pose);
        
        pose = calcNewPose(3, 0, -robotWidth, 0);
        tagArray[2] = new TagApproach(3, Alliance.Red, gameTarget.Processor, pose);
        
        pose = calcNewPose(4, robotWidth, 0, 0);
        tagArray[3] = new TagApproach(4, Alliance.Blue, gameTarget.Barge, pose);
        
        pose = calcNewPose(5, robotWidth, 0, 0);
        tagArray[4] = new TagApproach(5, Alliance.Red, gameTarget.Barge, pose);
        
        poseOffsetx = robotWidth * Math.cos(getTagAngle(6));
        poseOffsety = robotWidth * Math.sin(getTagAngle(6));
        // pose = calcNewPose(6, poseOffsetx, poseOffsety, 0);
        pose = calcNewPose(13.96, 2.64, 120);

        tagArray[5] = new TagApproach(6, Alliance.Red, gameTarget.Reef, pose);
        
        // pose = calcNewPose(7, robotWidth, 0, 0);
        pose = calcNewPose(14.75, 3.97, 180);

        tagArray[6] = new TagApproach(7, Alliance.Red, gameTarget.Reef, pose);
        
        poseOffsetx = robotWidth * Math.cos(getTagAngle(8));
        poseOffsety = robotWidth * Math.sin(getTagAngle(8));
        // pose = calcNewPose(8, poseOffsetx, poseOffsety, 0);
        pose = calcNewPose(13.90, 5.36, -120);

        tagArray[7] = new TagApproach(8, Alliance.Red, gameTarget.Reef, pose);
        
        poseOffsetx = robotWidth * Math.cos(getTagAngle(9));
        poseOffsety = robotWidth * Math.sin(getTagAngle(9));
        // pose = calcNewPose(9, poseOffsetx, poseOffsety, 0);
        pose = calcNewPose(12.13, 5.42, -60);
        tagArray[8] = new TagApproach(9, Alliance.Red, gameTarget.Reef, pose);
        
        // pose = calcNewPose(10, -robotWidth, 0, 0);
        pose = calcNewPose(11.38, 3.95, 0);

        tagArray[9] = new TagApproach(10, Alliance.Red, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(11));
        poseOffsety = robotWidth * Math.sin(getTagAngle(11));
        // pose = calcNewPose(11, poseOffsetx, poseOffsety, 0);
        pose = calcNewPose(12.28, 2.52, 60);
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
        // pose = calcNewPose(17, poseOffsetx, poseOffsety, 0);
        pose = calcNewPose(FieldLayout.getFieldLength() - 13.93, 2.58, 60);
        tagArray[16] = new TagApproach(17, Alliance.Blue, gameTarget.Reef, pose);

        // pose = calcNewPose(18, -robotWidth, 0, 0);
        pose = calcNewPose(FieldLayout.getFieldLength() - 14.75, 3.96, 0);

        tagArray[17] = new TagApproach(18, Alliance.Blue, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(19));
        poseOffsety = robotWidth * Math.sin(getTagAngle(19));
        // pose = calcNewPose(19, poseOffsetx, poseOffsety, 0);
        pose = calcNewPose(FieldLayout.getFieldLength() - 13.93, 5.39, -60);

        tagArray[18] = new TagApproach(19, Alliance.Blue, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(20));
        poseOffsety = robotWidth * Math.sin(getTagAngle(20));
        // pose = calcNewPose(20, poseOffsetx, poseOffsety, 0);
        pose = calcNewPose(FieldLayout.getFieldLength() - 12.205, 5.39, -120);

        tagArray[19] = new TagApproach(20, Alliance.Blue, gameTarget.Reef, pose);

        // pose = calcNewPose(21, robotWidth, 0, 0);
        pose = calcNewPose(FieldLayout.getFieldLength() - 11.38, 3.96, 180);
        tagArray[20] = new TagApproach(21, Alliance.Blue, gameTarget.Reef, pose);

        poseOffsetx = robotWidth * Math.cos(getTagAngle(22));
        poseOffsety = robotWidth * Math.sin(getTagAngle(22));
        // pose = calcNewPose(22, poseOffsetx, poseOffsety, 0);
        pose = calcNewPose(FieldLayout.getFieldLength() - 12.205, 2.58, 120);
        tagArray[21] = new TagApproach(22, Alliance.Blue, gameTarget.Reef, pose);

        // numbers past this point are not tags, but rather user specifified positions
        // not used in competition, but exists as a proof of concept
        // pose = calcNewPose(4, 7, 0);
        // tagArray[22] = new TagApproach("testPose1", Alliance.Red, gameTarget.Reef, pose);
    }

    private Pose2d calcNewPose(int id, double arbX, double arbY, double arbAngle) {
        Pose2d tagPose = FieldLayout.getTagPose(id).get().toPose2d();

        return new Pose2d(tagPose.getX() + arbX,
                tagPose.getY() + arbY,
                new Rotation2d(tagPose.getRotation().getRadians() + Math.toRadians(arbAngle) + Math.PI));
    }

    /* used to return a pose when the goal position is not measured from an April Tag */
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

        /* also exists for the unassiated pose rotation */
        // Alliance alliance = Robot.getInstance().m_Vision.MyAlliance();
        // if (indexInArray > 21 && alliance != null && alliance != tagArray[indexInArray].TagAlliance()) {
            
        //    return RotatePose2d(indexInArray);
                    
        // }
        
        Pose2d goalPose = tagArray[indexInArray].DesiredPos();

        if (tagArray[indexInArray].GameTarget() == gameTarget.Reef){ 
            System.out.println("shifting");
            return shiftReefAllign(goalPose);
        }
        return goalPose;
    }

    public Pose2d TagFieldPose2d(int tagID) {
        return FieldLayout.getTagPose(tagID).get().toPose2d();
    }

    public double getTagAngle(int tagID) {
        return FieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getDegrees();
    }

    /* un-finished test method to rotate poses not associated with tags around the center of the field */
    // public Pose2d RotatePose2d(int arrayIndex) {
    //     Pose2d oppOrigin = new Pose2d(FieldLayout.getFieldLength(), FieldLayout.getFieldWidth(), new Rotation2d(Math.PI));
    //     return tagArray[arrayIndex].DesiredPos().relativeTo(oppOrigin);
    // }
    
    // public Pose2d addTagCentricOffset(Pose2d goalBeforeShift, Pose2d offsetTagRelative) { //goalBeforeShift if field relative ||| offsetTagRelative is tagRelative
    //     System.out.println("start");
    //     System.out.println("start");
    //     System.out.println("start");
    //     double magnitudeOfOffset = offsetTagRelative.getTranslation().getDistance(new Translation2d());
    //     Rotation2d TagAngle = goalBeforeShift.getRotation();
    //     double offsetXBeforeShift = offsetTagRelative.getX();
    //     double offsetYBeforeShift = offsetTagRelative.getY();
    //     Rotation2d offsetAngleAwayFromTagToStandardPosition = new Rotation2d(Math.atan2(offsetYBeforeShift, offsetXBeforeShift));
    //     Rotation2d finalRefAngle = TagAngle.minus(new Rotation2d(90)).plus(offsetAngleAwayFromTagToStandardPosition);
    //     Translation2d newOffsetTranslation = new Translation2d(magnitudeOfOffset, finalRefAngle);
    //     Pose2d finalpose = new Pose2d(newOffsetTranslation, goalBeforeShift.getRotation());
    //     System.out.println("goal before shift: " + goalBeforeShift.toString());
    //     System.out.println("magnitudeOfOffset" + magnitudeOfOffset);
    //     System.out.println("tagAngle: " + TagAngle.toString());
    //     System.out.println("goalXBeforeShift" + offsetXBeforeShift);
    //     System.out.println("goalYBeforeShift" + offsetYBeforeShift);
    //     System.out.println("offsetAngleRefPos: " + offsetAngleAwayFromTagToStandardPosition.toString());
    //     System.out.println("finalRefAngle: " + finalRefAngle.toString());
    //     System.out.println("OffsetTranslation: " + newOffsetTranslation.toString());
    //     System.out.println(finalpose.toString());
    //     System.out.println("end");
    //     System.out.println("end");
    //     System.out.println("end");

    //     return finalpose;
    // }

    public Pose2d shiftReefAllign(Pose2d goalBeforeShift) {
        double offset = 0;

        if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.left) {
            offset = 0.327 / 2.0;
            System.out.println("moving left");
        } else if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.right) {
            offset = -0.327 / 2.0;
            System.out.println("moving right");
        } else {
            offset = 0;
            System.out.println("staying in the center");
            
        }

        Rotation2d goalAngle = goalBeforeShift.getRotation();
        Translation2d oldTranslation = goalBeforeShift.getTranslation();
        Translation2d offsetTranslation = new Translation2d(offset, goalAngle.plus(Rotation2d.fromDegrees(90)));
        Translation2d newGoalTranslation = oldTranslation.plus(offsetTranslation);

        return new Pose2d(newGoalTranslation, goalAngle);
    }

}