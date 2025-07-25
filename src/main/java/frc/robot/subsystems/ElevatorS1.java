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

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
public class ElevatorS1 extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public TalonFX stage1motor;
    // public TalonFX stage2motor;
    private DigitalInput elevatorBottomSwitch;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // public boolean enabledClimb = false;

    // TalonFXConfiguration elevatorUpperConfig = new TalonFXConfiguration();
    TalonFXConfiguration elevatorLowerConfig = new TalonFXConfiguration();
    // private final MotionMagicVoltage m_motionMagicReqU = new MotionMagicVoltage(0);
    private final MotionMagicVoltage m_motionMagicReqL = new MotionMagicVoltage(0);
    // public boolean stopped = false;
    public double elevatorStage1Target;
    // public double elevatorStage2Target;

    /**
    *
    */
    public ElevatorS1() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        stage1motor = new TalonFX(14);
        // stage2motor = new TalonFX(15);
        elevatorBottomSwitch = new DigitalInput(0);
        // FeedbackConfigs elevatorUpperFeedback = elevatorUpperConfig.Feedback;
        FeedbackConfigs elevatorLowerFeedback = elevatorLowerConfig.Feedback;
        // elevatorUpperFeedback.SensorToMechanismRatio = 45;
        elevatorLowerFeedback.SensorToMechanismRatio = 20;
        CurrentLimitsConfigs elevatorLowerCurrent = elevatorLowerConfig.CurrentLimits;
        // CurrentLimitsConfigs elevatorUpperCurrent = elevatorUpperConfig.CurrentLimits;
        elevatorLowerCurrent.StatorCurrentLimit = 25;
        // elevatorUpperCurrent.StatorCurrentLimit = 25;
        // SoftwareLimitSwitchConfigs elevatorLowerSoftSwitch = elevatorLowerConfig.SoftwareLimitSwitch;
        // elevatorLowerSoftSwitch.ForwardSoftLimitEnable = true;
        // elevatorLowerSoftSwitch.ForwardSoftLimitThreshold = 2.5;
        MotorOutputConfigs elevatorLowerOutput = elevatorLowerConfig.MotorOutput;
        elevatorLowerOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorLowerOutput.NeutralMode = NeutralModeValue.Brake;
        // MotorOutputConfigs elevatorUpperOutput = elevatorUpperConfig.MotorOutput;
        // elevatorUpperOutput.Inverted = InvertedValue.Clockwise_Positive;
        // elevatorUpperOutput.NeutralMode = NeutralModeValue.Brake;
        
        /* Configure Motion Magic */
        // MotionMagicConfigs motionMagicU = elevatorUpperConfig.MotionMagic;
        // motionMagicU.withMotionMagicCruiseVelocity(RotationsPerSecond.of(9999)) // (meachanism) rotations per second cruise
                // .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10000))
                // .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

        MotionMagicConfigs motionMagicL = elevatorLowerConfig.MotionMagic;
        motionMagicL.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10)) // (meachanism) rotations per second cruise
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));


        // Slot0Configs upperSlot0 = elevatorUpperConfig.Slot0;
        // upperSlot0.kP = 25; // A position error of 0.2 rotations results in 12 V output
        // upperSlot0.kI = 0.25; // No output for integrated error
        // upperSlot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        // upperSlot0.kV = 5.1659; // A velocity target of 1 rps results in 0.12 V output
        // upperSlot0.kS = 0.051841; // Add 0.25 V output to overcome static friction
        // upperSlot0.kA = 0.095406; // An acceleration of 1 rps/s requires 0.01 V output

        Slot0Configs lowerSlot0 = elevatorLowerConfig.Slot0;
        lowerSlot0.kP = 60; // A position error of 0.2 rotation/s results in 12 V output
        lowerSlot0.kI = 0; // No output for integrated error
        lowerSlot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
        lowerSlot0.kV = 5.226; // A velocity target of 1 rps results in 0.12 V output
        lowerSlot0.kS = 0.15296; // Add 0.25 V output to overcome static friction
        lowerSlot0.kA = 0.055325; // An acceleration of 1 rps/s requires 0.01 V output

        // StatusCode statusU = StatusCode.StatusCodeNotInitialized;
        // for (int i = 0; i < 5; ++i) {
            // statusU = stage2motor.getConfigurator().apply(elevatorUpperConfig);
            // if (statusU.isOK())
                // break;
        // }
        // if (!statusU.isOK()) {
            // System.out.println("Could not configure device. Error: " + statusU.toString());
        // }

        StatusCode statusL = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusL = stage1motor.getConfigurator().apply(elevatorLowerConfig);
            if (statusL.isOK())
                break;
        }
        if (!statusL.isOK()) {
            System.out.println("Could not configure device. Error: " + statusL.toString());
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Bottom Vol", stage1motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Pos", stage1motor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Top Vol", stage2motor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Top Pos", stage2motor.getPosition().getValueAsDouble());
        // SmartDashboard.putBoolean("Stage 2", Robot.getInstance().getTopStage2());

//         if (getBottomSwitch() && stage1motor.getPosition().getValueAsDouble()!= Constants.ElevatorConstants.stage1LowerLimit) {
//             stage1motor.setPosition(Constants.ElevatorConstants.stage1LowerLimit);

//         }
//         if (getTopSwitch() && Math.abs(stage2motor.getPosition().getValueAsDouble() - Constants.ElevatorConstants.stage2UpperLimit) < 0.1) {
//             stage2motor.setPosition(Constants.ElevatorConstants.stage2UpperLimit);
//         }

        SmartDashboard.putBoolean("bottomSwitch", getBottomSwitch());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public boolean getBottomSwitch() {
        return elevatorBottomSwitch.get();
    }

    // public boolean getTopSwitch() {
        // return Robot.getInstance().getTopStage2();
    // }

    public boolean isMotorOneAtPos() {
    
        return Math.abs(stage1motor.getPosition().getValueAsDouble() - elevatorStage1Target) < .1;
    }

    // public boolean isMotorTwoAtPos() {
        // return Math.abs(stage2motor.getPosition().getValueAsDouble() - elevatorStage2Target) < .1;
    // }

    public void setElevatorPosition() {
        // if (isSafeToMoveElevator()){
        stage1motor.setControl(
                m_motionMagicReqL.withPosition(elevatorStage1Target).withSlot(0));
        // stage2motor.setControl(
                // m_motionMagicReqU.withPosition(elevatorStage2Target).withSlot(0));
            
    } 

    // public void setClimb() {
        // if (Robot.getInstance().accessory.getRightTriggerAxis() >= .5) {
            // stage1motor.setControl(
                    // m_motionMagicReqL.withPosition(elevatorStage1Target).withSlot(0));
            // stage2motor.setControl(
                    // m_motionMagicReqU.withPosition(elevatorStage2Target).withSlot(0));
        // }
    // }

    // public void stopBothMotors() {
        // stage1motor.set(0);
        // stage2motor.set(0);
    // }

    public void stopMotor() {
        stage1motor.set(0);
    }

    public void setElevatorZeroingS1(){
        if (getBottomSwitch()) {
            stage1motor.set(0);
        } else {
            stage1motor.set(-0.25);
        }
    }

    public void setElevatorHomingS1(){
        if (!getBottomSwitch()) {
            stage1motor.set(0);
        } else {
            stage1motor.set(0.1);
        }
    }

    // public void setElevatorZeroingS2(){
        // if (getTopSwitch()) {
        //    stage2motor.set(0);
        // } else {
            // stage2motor.set(0.2);
        // }
    // }

    // public void setElevatorHomingS2(){
        // if (!getTopSwitch()) {
        //    stage2motor.set(0);
        // } else {
            // stage2motor.set(-0.1);
        // }
    // }

    // public void increase(){
        // if (elevatorStage2Target + 0.1f <= Constants.ElevatorConstants.stage2UpperLimit)
            // elevatorStage2Target += 0.1f;
    // }

    // public void decrease(){
        // if (elevatorStage2Target - 0.1f >= Constants.ElevatorConstants.stage2LowerLimit)
            // elevatorStage2Target -= 0.1f;
    // }

    // public boolean isSafeToMoveElevator() {
    //     double currPos = Robot.getInstance().m_shoulder.shoulderMotor.getPosition().getValueAsDouble();
    //     double quadrant = Constants.ShoulderConstants.shoulderUpperLimit / 4;
    //     double safeLower = Constants.ShoulderConstants.shoulderLowerLimit + quadrant;
    //     double safeUpper = Constants.ShoulderConstants.shoulderUpperLimit - quadrant;
    //     return (currPos >= safeLower && currPos <= safeUpper);
    // } 

    public void setStage1MotorPosition(double position){
        stage1motor.setPosition(position);
    }

    // public void setStage2MotorPosition(double position){
        // stage2motor.setPosition(position);
    // }
    
}
