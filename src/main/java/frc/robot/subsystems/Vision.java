package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    
    // public AprilTagFieldLayout aprilTag_FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // public Pose2d getVisionCalculatedRobotPose = new Pose2d();

    private static final Vision m_Vision = new Vision();
//
    // PID values and supporting storage for turning towards targets
    // private double _turnkp = .0065;
    // private double _turnki = 0.0;
    // private double _turnkd = 0;

    // private double _rotkp = .0075;
    // private double _rotki = 0.0;
    // private double _rotkd = 0;

    // private PIDController _turnToTargetPID = new PIDController(_turnkp, _turnki, _turnkd);
    // private PIDController _rotateToTargetPID = new PIDController(_rotkp, _rotki, _rotkd);
    // private double turnPower = 0;
    // private double rotPower = 0;


   // private String _limelightName = "limelight-tags";

    // Supplier of pose information for each pose.
    // private TagApproaches _tagApproches;

    public static Vision getInstance() {
        return m_Vision;
    }

    public Vision() {
        // _tagApproches = new TagApproaches();

        // // Set tolerance to 2 degrees
        // _turnToTargetPID.setTolerance(2);
        // _rotateToTargetPID.setTolerance(2);

        LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.limeLightName, -0.073, 0, 0.44, 0, 0, 0);
        LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.limeLightName2, -0.093, 0, 0.44, 0, 21, 180);
    }

    private Pose2d currentOptimalPose;

    @Override
    public void periodic() {
        // Periodically, update the data on the current target
        // UpdateTargetData();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // private void UpdateTargetData() {
    //     boolean aquired = AllianceTargetAquired();
    //     if (aquired) {
    //         int targetID = (int) LimelightHelpers.getFiducialID(_limelightName);
    //         CalculateStearingValues(targetID);
    //         this.currentOptimalPose = _tagApproches.DesiredRobotPos(targetID);
    //     }
    // }

    public Alliance MyAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red ? Alliance.Red : Alliance.Blue;
        } else {
            return null;
        }
    }

    private void CalculateStearingValues(int targetID) {
        // // using data from tag approaches, determine the desired pose for the target
        // // currently be tracked
        // /// TO DO - Calculate the desired pose for the target we are tracking. Do this
        // // only if the target is ours.///
        // /// otherwise return null??? or something to that affect.
        // this.currentOptimalPose = new Pose2d(0, 0, new Rotation2d(0));

        // /// other calculations for PID turning to target may be appropriate here as
        // /// well.
        // turnPower = _turnToTargetPID.calculate(LimelightHelpers.getTX(_limelightName), 0);
        // rotPower = _rotateToTargetPID.calculate(Units.radiansToDegrees(LimelightHelpers.getBotPose3d_TargetSpace(_limelightName).getRotation().getY()),0);
        // if (_turnToTargetPID.atSetpoint())
        //     turnPower = 0;
        
        // if (_rotateToTargetPID.atSetpoint())
        //     rotPower = 0;
    }

    // public Pose2d GetTargetPose() {
    //     // return this.currentOptimalPose;
    // }

    // public boolean AllianceTargetAquired() {
        // boolean targetAquired = LimelightHelpers.getTV(_limelightName);
        // if (targetAquired) {
        //     int targetID = (int) LimelightHelpers.getFiducialID(_limelightName);
        //     if ((targetID >= 0) && (targetID <= 16))
        //         return (MyAlliance() == _tagApproches.TagAlliance(targetID));
        //     else
        //         return false;
        // }
        // return false;
    // }

    // public double GetTargetTurnPower() {
    //     // return turnPower;
    // }

    // public double GetTargetRotPower() {
    //     // return -rotPower;
    // }
}
