// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Shoulder extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // private final NeutralOut m_brake = new NeutralOut();
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private TalonFX shoulderMotor;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final MotionMagicVoltage m_motionMagicReq = new MotionMagicVoltage(0).withSlot(0);
    public double shoulderTarget;
    TalonFXConfiguration shoulderConf = new TalonFXConfiguration();

    /**
    *
    */
    public Shoulder() {
        shoulderMotor = new TalonFX(16, "rio");
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // Configs
        
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // Current Limits
        configs.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(4))
                        .withStatorCurrentLimitEnable(true));
        configs.withMotorOutput(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive));

        configs.Slot0.kP = Constants.ShoulderConstants.P; // An error of 1 rotation results in 2.4 V output
        configs.Slot0.kI = Constants.ShoulderConstants.I; // No output for integrated error
        configs.Slot0.kD = Constants.ShoulderConstants.D; // A velocity of 1 rps results in 0.1 V output

        // Peak output of 8 V
        configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        MotionMagicConfigs motionMagicOn = configs.MotionMagic;
        motionMagicOn.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        FeedbackConfigs shoulderFeed = shoulderConf.Feedback;
       // shoulderFeed.SensorToMechanismRatio = 270;
        MotorOutputConfigs shoulderOutput = shoulderConf.MotorOutput;
        shoulderOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shoulderOutput.NeutralMode = NeutralModeValue.Brake;

        // Config 
        
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shoulderMotor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (isShoulderTripped() && shoulderMotor.getPosition().getValueAsDouble() != 0) {
            shoulderMotor.setPosition(0);
        }

        SmartDashboard.putNumber("Shoulder Position", shoulderMotor.getPosition().getValueAsDouble());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void shoulderMove() {
        if (Robot.getInstance().m_elevator.isSafeToMoveShoulder(Robot.getInstance().currentArrangementPlacing()))
            shoulderMotor.setControl(m_motionMagicReq.withPosition(shoulderTarget));
    }

    public void stopShoulder() {
        shoulderMotor.set(0);
    }

    public boolean isShoulderTripped() {
        return Robot.getInstance().getShoulderTripped();
    }

    public boolean isShoulderAtGoalPos() {
        SmartDashboard.putBoolean("isShoulderAtGoal",
                Math.abs(shoulderMotor.getPosition().getValueAsDouble() - shoulderTarget) < 1 / (365 * 2));

        return Math.abs(shoulderMotor.getPosition().getValueAsDouble() - shoulderTarget) < 1.0;
    }

    public void setShoulderZeroing() {
        if (isShoulderTripped()) {
            shoulderMotor.set(0);
        } else {
            shoulderMotor.set(-0.25);
        }

    }

    public boolean isSafeToMoveWrist() {
        double currPos = shoulderMotor.getPosition().getValueAsDouble();
        double quadrant = Constants.ShoulderConstants.shoulderUpperLimit / 4;
        double safeLower = Constants.ShoulderConstants.shoulderLowerLimit + quadrant;
        double safeUpper = Constants.ShoulderConstants.shoulderUpperLimit - quadrant;
        return (currPos >= safeLower && currPos <= safeUpper);
    }

}
