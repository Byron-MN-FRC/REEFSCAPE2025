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

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    public TalonFX shoulderMotor;
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
        CurrentLimitsConfigs shoulderCurrent = shoulderConf.CurrentLimits;
        shoulderCurrent.StatorCurrentLimit = 25;
        MotorOutputConfigs shoulderOutput = shoulderConf.MotorOutput;
        shoulderOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shoulderOutput.NeutralMode = NeutralModeValue.Brake;
  
        shoulderConf.Slot0.kP = 0.75; // An error of 1 rotation results in 2.4 V output
        shoulderConf.Slot0.kI = 0.0075; // No output for integrated error
        shoulderConf.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output
        shoulderConf.Slot0.kS = 0.17558;
        shoulderConf.Slot0.kV = 0.11202;
        shoulderConf.Slot0.kA = 0.0024753;

        shoulderConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Peak output of 8 V
        shoulderConf.Voltage.withPeakForwardVoltage(Volts.of(10)).withPeakReverseVoltage(Volts.of(-10));

        MotionMagicConfigs motionMagicOn = shoulderConf.MotionMagic;
        motionMagicOn.withMotionMagicCruiseVelocity(RotationsPerSecond.of(9999))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10000))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0)); // 0 makes it fast
        
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shoulderMotor.getConfigurator().apply(shoulderConf);
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
        SmartDashboard.putBoolean("ShoulderTripped", Robot.getInstance().getShoulderTripped());
        SmartDashboard.putNumber("Shoulder Position", shoulderMotor.getPosition().getValueAsDouble());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void shoulderMove() {
        // if (isSafeToMoveShoulder()){
        shoulderMotor.setControl(m_motionMagicReq.withPosition(shoulderTarget));
    }

    public void stopShoulder() {
        shoulderMotor.set(0);
    }

    public boolean isShoulderTripped() {
        return Robot.getInstance().getShoulderTripped();
    }

    public boolean isShoulderAtGoalPos() {

        return Math.abs(shoulderMotor.getPosition().getValueAsDouble() - shoulderTarget) < .3;
    }

    public void setShoulderZeroing() {
        if (isShoulderTripped()) {
            shoulderMotor.set(0);
        } else {
            shoulderMotor.set(-0.2);
        }

    }

    public void setShoulderHoming(){
        if (!isShoulderTripped()) {
            shoulderMotor.set(0);
        } else {
            shoulderMotor.set(0.05);
        }
    }

    public void setShoulderMotorPosition(double position){
        shoulderMotor.setPosition(position);
    }

    // public void setShoulderClimb() {
    //     if (Robot.getInstance().accessory.getRightTriggerAxis() >= .5) {
    //         shoulderMotor.setControl(
    //                 m_motionMagicReq.withPosition(shoulderTarget).withSlot(0));
    //     }
    // }

    // public boolean isSafeToMoveShoulder() {
    //     double currPos = Robot.getInstance().m_elevator.stage2motor.getPosition().getValueAsDouble();
    //     double quadrant = Constants.ElevatorConstants.stage2UpperLimit / 4;
    //     double safeLower = Constants.ElevatorConstants.stage2LowerLimit + quadrant;
    //     // double safeUpper = Constants.ElevatorConstants.stage2UpperLimit - quadrant;
    //     return (currPos >= safeLower);
    // } 
}
