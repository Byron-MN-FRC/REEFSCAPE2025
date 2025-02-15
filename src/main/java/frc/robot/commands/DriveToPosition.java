package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.TagApproaches;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveToPosition extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Magnitude_Constraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private String _limelightName = Constants.VisionConstants.limeLightName;

    private TagApproaches tagApproaches = TagApproaches.getInstance();
    private final Drivetrain drivetrain;
    private Pose2d goalPose;
    private double initialR;
    private double angle;

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController magnitudeController = new ProfiledPIDController(2.5, 0, 0, Magnitude_Constraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(3, 0, .1, OMEGA_CONSTRAINTS);

    private int lastTarget;
/* old up */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public DriveToPosition(Drivetrain subsystem) {
        drivetrain = subsystem;

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        omegaController.setTolerance(Units.degreesToRadians(2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        // magnitudeController.setTolerance(0.05);

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //ensure the active pipeline can process april tags
        // if (LimelightHelpers.getCurrentPipelineType(_limelightName) != "pipe_fiducial") {
        //     LimelightHelpers.setPipelineIndex(_limelightName, Constants.aprilPipe);
        // }

        // Set position of camera based on target seen 
        lastTarget = 0;
        // Verify that see a valid target for aliance and set current robot pose based on it.
          if (LimelightHelpers.getTV("")) {
            int fidID = (int) LimelightHelpers.getFiducialID(_limelightName);
            if ((fidID >= 0) && (fidID <= 22)) {
                lastTarget = fidID;
                
                // goalPose = TagApproaches.getInstance().DesiredRobotPos(10);

                // SmartDashboard.putString("goal pose", goalPose.toString());
            }
        }

        goalPose = tagApproaches.DesiredRobotPos(Robot.getInstance().globalCurrNumSelected);
        initialR = goalPose.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation());

        omegaController.reset(drivetrain.getState().Pose.getRotation().getRadians());
        yController.reset(drivetrain.getState().Pose.getY());
        xController.reset(drivetrain.getState().Pose.getX());
        magnitudeController.reset(drivetrain.getState().Pose.getTranslation().getDistance(goalPose.getTranslation()));
        System.out.println("yo this works");
        
        Robot.getInstance().targetPoseField.setRobotPose(goalPose);
    }

    // // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

            //update polar coords
            double currentR = drivetrain.getState().Pose.getTranslation().getDistance(goalPose.getTranslation());
            double distCxGx = drivetrain.getState().Pose.getTranslation().getX() - goalPose.getTranslation().getX();
            if (drivetrain.getState().Pose.getY() > goalPose.getY()) {
                angle = Math.toDegrees(-1.0 * Math.acos(distCxGx / currentR));
                SmartDashboard.putBoolean("invertS", true);
            } else {
                angle = Math.toDegrees(Math.acos(distCxGx / currentR));
                SmartDashboard.putBoolean("invertS", false);
            }
            SmartDashboard.putNumber("angleRJIEOFOS", angle);
            SmartDashboard.putNumber("cR", currentR);
            SmartDashboard.putNumber("dCXGX", distCxGx);

            // Drive
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
            magnitudeController.setGoal(0);

            // Drive to the target
            var xSpeed = xController.calculate(drivetrain.getState().Pose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            var ySpeed = yController.calculate(drivetrain.getState().Pose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            var omegaSpeed = omegaController.calculate(drivetrain.getState().Pose.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }

            // var combinedSpeed = magnitudeController.calculate(currentR);
                
            // double xSpeedFromPolar = -1 * Math.cos(angle) * combinedSpeed;
            // double ySpeedFromPolar = -1 * Math.sin(angle) * combinedSpeed;
            
            // if (magnitudeController.atGoal()) {
            //     xSpeedFromPolar = 0;
            //     ySpeedFromPolar = 0;
            // }



        // drivetrain.setControl(
        //     Robot.getInstance().drive
        //         .withVelocityX(xSpeedFromPolar * MaxSpeed)
        //         .withVelocityY(ySpeedFromPolar * MaxSpeed)
        //         .withRotationalRate(omegaSpeed * MaxAngularRate)
        // );
        
        drivetrain.setControl(
            Robot.getInstance().drive
                .withVelocityX(xSpeed * MaxSpeed)
                .withVelocityY(ySpeed * MaxSpeed)
                .withRotationalRate(omegaSpeed * MaxAngularRate)
        );

        // System.out.println("Last Taget: " + lastTarget);
        // System.out.println();

        // System.out.println("Goal Pose X: " + goalPose.getX());
        // System.out.println("Goal Pose Y: " + goalPose.getY());
        // System.out.println("Goal Pose R: " + goalPose.getRotation());
        // System.out.println();
        
        // System.out.println("X Speed: " + xSpeed);
        // System.out.println("Y Speed: " + ySpeed);
        // System.out.println("R Speed: " + omegaSpeed);
        // System.out.println();
        // System.out.println("this also works");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
//cancerl current trajectory
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
