package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.Meter;

public class SocialDistancing extends Command {
    
    private CommandSwerveDrivetrain m_drivetrain;
    private AlignmentSubsystem m_distSensor;

    private static final TrapezoidProfile.Constraints FORWARD_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 1);

    private final ProfiledPIDController forwardController = new ProfiledPIDController(2.0, 0, 0, FORWARD_CONSTRAINTS);
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity    

    public final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public SocialDistancing (CommandSwerveDrivetrain drivetrain, AlignmentSubsystem alignmentSubsystem) {
        forwardController.setTolerance(0.003);
        m_drivetrain = drivetrain;
        m_distSensor = alignmentSubsystem;
        addRequirements(drivetrain, alignmentSubsystem);
    }

    
    @Override
    public void initialize() {
        forwardController.reset(m_distSensor.getDistance());
    }

    @Override
    public void execute() {
        // SmartDashboard.putBoolean("atTarget", forwardController.atGoal());
        // Drive
        forwardController.setGoal(0.305);

        double forwardSpeed = forwardController.calculate(m_distSensor.getDistance());


        // SmartDashboard.putNumber("forward speed", forwardSpeed);
        // SmartDashboard.putNumber("distance from goal", m_distSensor.getDistance());
        if (forwardController.atGoal()) {
            forwardSpeed = 0;
        }

        m_drivetrain.setControl(
            driveRobotCentric
                .withVelocityX(forwardSpeed * MaxSpeed)
                .withVelocityY(Robot.getInstance().joystick.getLeftX() * MaxSpeed * 0.2)
                .withRotationalRate(0)
        );
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
