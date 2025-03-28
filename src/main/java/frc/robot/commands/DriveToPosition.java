package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.TagApproaches;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveToPosition extends Command {

    private static final TrapezoidProfile.Constraints Magnitude_Constraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    
    private final ProfiledPIDController magnitudeController = new ProfiledPIDController(2.75, 0, 0, Magnitude_Constraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(4, 0, .1, OMEGA_CONSTRAINTS);

    private String _limelightName = Constants.VisionConstants.limelightName;
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d goalPose;
    private double angle;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity    
    
    public DriveToPosition(CommandSwerveDrivetrain subsystem, String llName) {
        drivetrain = subsystem;
        _limelightName = llName;
        omegaController.setTolerance(Units.degreesToRadians(1.5));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        magnitudeController.setTolerance(0.04);

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        if (_limelightName == Constants.VisionConstants.limelightName) {
            goalPose = TagApproaches.getInstance().DesiredRobotPos(Vision.getInstance().lastTargetFront);
        }
        if (_limelightName == Constants.VisionConstants.limelightName2) {
            goalPose = TagApproaches.getInstance().DesiredRobotPos(Vision.getInstance().lastTargetBack);
        }

        // SmartDashboard.putString("goal pose", goalPose.toString());

        // SmartDashboard.putString("currentPose", drivetrain.getState().Pose.toString());
        omegaController.reset(drivetrain.getState().Pose.getRotation().getRadians());
        magnitudeController.reset(drivetrain.getState().Pose.getTranslation().getDistance(goalPose.getTranslation()));
        
        Robot.getInstance().targetPoseField.setRobotPose(goalPose);
    }

    // // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //update polar coords
        double currentR = drivetrain.getState().Pose.getTranslation().getDistance(goalPose.getTranslation());
        double distCxGx = goalPose.getTranslation().getX() - drivetrain.getState().Pose.getTranslation().getX();
        if (goalPose.getY() < drivetrain.getState().Pose.getY()) {
            angle = -1.0 * Math.acos(distCxGx / currentR);
            
        } else {
            angle = Math.acos(distCxGx / currentR);
        }

        // Drive
        omegaController.setGoal(goalPose.getRotation().getRadians());
        magnitudeController.setGoal(0);

        var omegaSpeed = omegaController.calculate(drivetrain.getState().Pose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        var combinedSpeed = magnitudeController.calculate(currentR);
            
        double xSpeedFromPolar = -1 * Math.cos(angle) * combinedSpeed;
        double ySpeedFromPolar = -1 * Math.sin(angle) * combinedSpeed;
        
        if (magnitudeController.atGoal()) {
            xSpeedFromPolar = 0;
            ySpeedFromPolar = 0;
        }

        if (Robot.DRIVE_TO_POSITION_DEBUG) {
        SmartDashboard.putNumber("combinedSpeed", combinedSpeed);
        SmartDashboard.putNumber("xSpeedFromPolar", xSpeedFromPolar);
        SmartDashboard.putNumber("ySpeedFromPolar", ySpeedFromPolar);
        SmartDashboard.putNumber("angleRJIEOFOS", angle * (180/Math.PI));
        SmartDashboard.putNumber("cR", currentR);
        SmartDashboard.putNumber("dCXGX", distCxGx);
        }

        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.get() == Alliance.Blue) {
            drivetrain.setControl(
            Robot.getInstance().drive
            .withVelocityX(xSpeedFromPolar * MaxSpeed)
            .withVelocityY(ySpeedFromPolar * MaxSpeed)
            .withRotationalRate(omegaSpeed * MaxAngularRate)
            );
        } else {
            drivetrain.setControl(
            Robot.getInstance().drive
            .withVelocityX(-xSpeedFromPolar * MaxSpeed)
            .withVelocityY(-ySpeedFromPolar * MaxSpeed)
            .withRotationalRate(omegaSpeed * MaxAngularRate)
            );
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return magnitudeController.atGoal() && omegaController.atGoal();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
