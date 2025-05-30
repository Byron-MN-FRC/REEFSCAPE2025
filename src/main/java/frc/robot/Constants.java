// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final class Selector {
        public static final class DPAD {
            public static final int kUp = 0;
            public static final int kRight = 90;
            public static final int kDown = 180;
            public static final int kLeft = 270;
        }

        public static class PlacementSelector {

            private static boolean[][] array = new boolean[4][2];
            private static int currentRow = 0;
            private static int currentCol = 0;
            public static String level = "blank";
            public static String left = "left";
            public static String right = "right";

            public PlacementSelector() {

                // Initially set the first element to true
                array[currentRow][currentCol] = true;
            }

            public static void move(int direction) {
                // Set the current true element to false
                array[currentRow][currentCol] = false;
                SmartDashboard.putBoolean(currentRow + "-" + currentCol, false);

                if (direction == Constants.Selector.DPAD.kDown && currentRow > 0) {
                    currentRow--;
                } else if (direction == Constants.Selector.DPAD.kUp && currentRow < array.length - 1) {
                    currentRow++;
                } else if (direction == Constants.Selector.DPAD.kLeft && currentCol > 0) {
                    currentCol--;
                } else if (direction == Constants.Selector.DPAD.kRight && currentCol < array[0].length - 1) {
                    currentCol++;
                }

                // Set the new position to true
                array[currentRow][currentCol] = true;
                SmartDashboard.putBoolean(currentRow + "-" + currentCol, true);
            }

            public static int getCurrentCol() {
                return currentCol;
            }

            public static int getCurrentRow() {
                return currentRow;
            }

            public static void setCurrentRow(int newRow) {
                // Set the current true element to false
                array[currentRow][currentCol] = false;
                SmartDashboard.putBoolean(currentRow + "-" + currentCol, false);

                if (newRow >= 0 && newRow <= (array.length -1)) {
                    currentRow = newRow;
                }

                // Set the new position to true
                array[currentRow][currentCol] = true;
                SmartDashboard.putBoolean(currentRow + "-" + currentCol, true);
            }

            public static void initializeTab() {
                for (int i = 0; i<array.length; i++){
                    for (int j = 0; j<array[i].length; j++){
                        array[i][j] = false;
                        SmartDashboard.putBoolean(i+"-"+j, false);
                    }
                }
                array[currentRow][currentCol] = true;
                SmartDashboard.putBoolean(currentRow+"-"+currentCol, true);

            }

            public static void printArray() {
                for (boolean[] row : array) {
                    for (boolean element : row) {
                        System.out.print(element + " ");
                    }
                    System.out.println();
                }
            }

            public static String getLevel(){
                // if (DriverStation.isAutonomous()) level = "L4";
                if (getCurrentRow() == 0) level = "L1";
                else if (getCurrentRow() == 1) level = "L2";
                else if (getCurrentRow() == 2) level = "L3";
                else if (getCurrentRow() == 3) level = "L4";
                return level;
            }

            public static String getScoringPose(){
                String side = "blank";
                if (getCurrentCol() == 0)side = left;
                else if (getCurrentCol() == 1)side = right;
                return side;
            }

        }
    }

    public static final class SwerveConstants {
        public static final double percentSlow = 0.35;
        public static final double smoothingFactor = 0.5;
        // Closer to 1: Faster changes (less smoothing), Closer to 0: Slower changes (more smoothing).
    } 

    public static final class ElevatorConstants {

        public static final double stage2UpperLimit = 3.02;
        public static final double stage2LowerLimit = 0;
        public static final double stage2SensorLimit = 3.08;

        public static final double stage1UpperLimit = 5.17;
        public static final double stage1LowerLimit = 0.06;
        public static final double stage1SensorLimit = 0;

    }

    public static final class ShoulderConstants {

        public static final double shoulderUpperLimit = 223.96;
        public static final double shoulderLowerLimit = 0.06;
        public static final double shoulderSensorLimit = 0;

    }

    public static final class ClawConstants {
    }

    public static final class VisionConstants {
        public static final String limelightName = "limelight-front";
        public static final String limelightName2 = "limelight-back";

        public static final double fieldWidth = 8.052;
        public static final double fieldLength = 17.548;
        
        public static final Pose2d ReefTagOffset = new Pose2d(0, Units.inchesToMeters(17.25), new Rotation2d(Math.PI));
        // public static final Pose2d CoralStationTagOffset = new Pose2d(0, Units.inchesToMeters(25), new Rotation2d()); 
        public static final Pose2d CoralStationTagOffset = new Pose2d(0, Units.inchesToMeters(27), new Rotation2d());
    }

}