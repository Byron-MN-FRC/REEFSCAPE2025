package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

        pose = calcNewPose(1, 0, 0, 0);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.CoralStationTagOffset);
        tagArray[0] = new TagApproach(1, Alliance.Red, gameTarget.CoralStation, pose);
        
        pose = calcNewPose(2, 0, 0, 0);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.CoralStationTagOffset);
        tagArray[1] = new TagApproach(2, Alliance.Red, gameTarget.CoralStation, pose);
        
        pose = calcNewPose(3, 0, -robotWidth, 0);
        tagArray[2] = new TagApproach(3, Alliance.Red, gameTarget.Processor, pose);
        
        pose = calcNewPose(4, robotWidth, 0, 0);
        tagArray[3] = new TagApproach(4, Alliance.Blue, gameTarget.Barge, pose);
        
        pose = calcNewPose(5, robotWidth, 0, 0);
        tagArray[4] = new TagApproach(5, Alliance.Red, gameTarget.Barge, pose);
        
        pose = calcNewPose(6, 0, 0, 0);
        // pose = calcNewPose(13.96, 2.64, 120);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[5] = new TagApproach(6, Alliance.Red, gameTarget.Reef, pose);
        
        pose = calcNewPose(7, 0, 0, 0);
        // pose = calcNewPose(14.75, 3.97, 180);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[6] = new TagApproach(7, Alliance.Red, gameTarget.Reef, pose);
        
        pose = calcNewPose(8, 0, 0, 0);
        // pose = calcNewPose(13.90, 5.36, -120);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[7] = new TagApproach(8, Alliance.Red, gameTarget.Reef, pose);
        
        pose = calcNewPose(9, 0, 0, 0);
        // pose = calcNewPose(12.13, 5.42, -60);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[8] = new TagApproach(9, Alliance.Red, gameTarget.Reef, pose);
        
        pose = calcNewPose(10, 0, 0, 0);
        // pose = calcNewPose(11.38, 3.95, 0);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[9] = new TagApproach(10, Alliance.Red, gameTarget.Reef, pose);

        pose = calcNewPose(11, 0, 0, 0);
        // pose = calcNewPose(12.28, 2.52, 60);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[10] = new TagApproach(11, Alliance.Red, gameTarget.Reef, pose);

        pose = calcNewPose(12, 0, 0, 0);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.CoralStationTagOffset);
        tagArray[11] = new TagApproach(12, Alliance.Blue, gameTarget.CoralStation, pose);

        pose = calcNewPose(13, poseOffsetx, poseOffsety, 0);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.CoralStationTagOffset);
        tagArray[12] = new TagApproach(13, Alliance.Blue, gameTarget.CoralStation, pose);

        pose = calcNewPose(14, -robotWidth, 0, 0);
        tagArray[13] = new TagApproach(14, Alliance.Blue, gameTarget.Barge, pose);

        pose = calcNewPose(15, -robotWidth, 0, 0);
        tagArray[14] = new TagApproach(15, Alliance.Red, gameTarget.Barge, pose);

        pose = calcNewPose(16, 0, robotWidth, 0);
        tagArray[15] = new TagApproach(16, Alliance.Blue, gameTarget.Processor, pose);

        pose = calcNewPose(17, 0, 0, 0);
        // pose = calcNewPose(FieldLayout.getFieldLength() - 13.93, 2.58, 60);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[16] = new TagApproach(17, Alliance.Blue, gameTarget.Reef, pose);

        pose = calcNewPose(18, 0, 0, 0);
        // pose = calcNewPose(FieldLayout.getFieldLength() - 14.75, 3.96, 0);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[17] = new TagApproach(18, Alliance.Blue, gameTarget.Reef, pose);

        pose = calcNewPose(19, 0, 0, 0);
        // pose = calcNewPose(FieldLayout.getFieldLength() - 13.93, 5.39, -60);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[18] = new TagApproach(19, Alliance.Blue, gameTarget.Reef, pose);

        pose = calcNewPose(20, 0, 0, 0);
        // pose = calcNewPose(FieldLayout.getFieldLength() - 12.205, 5.39, -120);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[19] = new TagApproach(20, Alliance.Blue, gameTarget.Reef, pose);

        pose = calcNewPose(21, 0, 0, 0);
        // pose = calcNewPose(FieldLayout.getFieldLength() - 11.38, 3.96, 180);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
        tagArray[20] = new TagApproach(21, Alliance.Blue, gameTarget.Reef, pose);

        pose = calcNewPose(22, 0, 0, 0);
        // pose = calcNewPose(FieldLayout.getFieldLength() - 12.205, 2.58, 120);
        pose = addTagCentricOffset(pose, Constants.VisionConstants.ReefTagOffset);
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
                new Rotation2d(tagPose.getRotation().getRadians()/* + Math.toRadians(arbAngle) + Math.PI*/));
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
            return shiftReefAllign(goalPose);
        }
        if (tagArray[indexInArray].GameTarget() == gameTarget.CoralStation){
            return shiftFeederAllign(goalPose);
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
    
    public Pose2d addTagCentricOffset(Pose2d goalBeforeShift, Pose2d offsetTagRelative) { //goalBeforeShift if field relative ||| offsetTagRelative is tagRelative

        Rotation2d TagAngle = goalBeforeShift.getRotation();
        Translation2d offsetTagRelativeTranslation = offsetTagRelative.getTranslation();
        Rotation2d offsetRotation = offsetTagRelative.getRotation();
        
        Translation2d TagTranslation = goalBeforeShift.getTranslation();
        Translation2d fieldOrientedOffset = offsetTagRelativeTranslation.rotateBy(TagAngle.minus(new Rotation2d(Math.PI / 2)));
        Translation2d newTranslation = TagTranslation.plus(fieldOrientedOffset);
        Pose2d newPose = new Pose2d(newTranslation, TagAngle.plus(offsetRotation));
        
        // if (Robot.VISIONTEST) {
        //     System.out.println("goalBS" + goalBeforeShift);
        //     System.out.println("offsetTR" + offsetTagRelative);
        //     System.out.println("tA" + TagAngle);
        //     System.out.println("tagT" + TagTranslation);
        //     System.out.println("fieldOO" + fieldOrientedOffset);
        //     System.out.println("newT" + newTranslation);
        // }
        
        return newPose;
    }

    public Pose2d shiftReefAllign(Pose2d goalBeforeShift) {
        double offset = 0;

        if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.left) {
            offset = 0.1234;
            // if (Robot.VISIONTEST) System.out.println("moving left");
        } else if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.right) {
            offset = -0.235;
            // if (Robot.VISIONTEST) System.out.println("moving right");
        } else {
            offset = 0;
            // if (Robot.VISIONTEST) System.out.println("staying in the center");
            
        }

        Rotation2d goalAngle = goalBeforeShift.getRotation();
        Translation2d oldTranslation = goalBeforeShift.getTranslation();
        Translation2d offsetTranslation = new Translation2d(offset, goalAngle.plus(Rotation2d.fromDegrees(90)));
        Translation2d newGoalTranslation = oldTranslation.plus(offsetTranslation);

        return new Pose2d(newGoalTranslation, goalAngle);
    }
    
    public Pose2d shiftFeederAllign(Pose2d goalBeforeShift) {
        double offset = 0;
        // closer is +-25.75, farther is +-15.75
        if (goalBeforeShift.getX() > Constants.VisionConstants.fieldLength / 2) {
            
            if (goalBeforeShift.getY() > Constants.VisionConstants.fieldWidth / 2) {
                // top right
                if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.left) {
                    offset = Units.inchesToMeters(25.75);
                } else if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.right) {
                    offset = Units.inchesToMeters(-15.75);
                }
            } else {
                // bottom right
                if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.left) {
                    offset = Units.inchesToMeters(15.75);
                } else if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.right) {
                    offset = Units.inchesToMeters(-25.75);
                }
            }

        } else {

            if (goalBeforeShift.getY() > Constants.VisionConstants.fieldWidth / 2) {
                // top left
                if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.left) {
                    offset = Units.inchesToMeters(15.75);
                } else if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.right) {
                    offset = Units.inchesToMeters(-25.75);
                }

            } else {
                // bottom left
                if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.left) {
                    offset = Units.inchesToMeters(25.75);
                } else if (Constants.Selector.PlacementSelector.getScoringPose() == Constants.Selector.PlacementSelector.right) {
                    offset = Units.inchesToMeters(-15.75);
                }
                
            }

        }

        Rotation2d goalAngle = goalBeforeShift.getRotation();
        Translation2d oldTranslation = goalBeforeShift.getTranslation();
        Translation2d offsetTranslation = new Translation2d(offset, goalAngle.plus(Rotation2d.fromDegrees(90)));
        Translation2d newGoalTranslation = oldTranslation.plus(offsetTranslation);

        return new Pose2d(newGoalTranslation, goalAngle);
    }
}