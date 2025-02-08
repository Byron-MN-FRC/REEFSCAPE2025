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

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import frc.robot.Constants;
import frc.robot.Robot;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Elevator extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private TalonFX stage1motor;
    private TalonFX stage2motor;
    private DigitalInput elevatorBottomSwich;
    private DigitalInput elevatorTopSwich;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public boolean enabledClimb = false;

    TalonFXConfiguration elevatorUpperConfig = new TalonFXConfiguration();
    TalonFXConfiguration elevatorLowerConfig = new TalonFXConfiguration();
    private final MotionMagicVoltage m_motionMagicReqU = new MotionMagicVoltage(0);
    private final MotionMagicVoltage m_motionMagicReqL = new MotionMagicVoltage(0);
    public boolean stopped = false;
    private double stage1motorGoalPos;
    private double stage2motorGoalPos;

    /**
    *
    */
    public Elevator() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        stage1motor = new TalonFX(14);
        stage2motor = new TalonFX(15);
        elevatorBottomSwich = new DigitalInput(9);
        elevatorTopSwich = new DigitalInput(8);
        FeedbackConfigs elevatorUpperFeedback = elevatorUpperConfig.Feedback;
        FeedbackConfigs elevatorLowerFeedback = elevatorLowerConfig.Feedback;
        elevatorUpperFeedback.SensorToMechanismRatio = 12.8;
        elevatorLowerFeedback.SensorToMechanismRatio = 12.8;

        /* Configure Motion Magic */
        MotionMagicConfigs motionMagicU = elevatorUpperConfig.MotionMagic;
        motionMagicU.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // (meachanism) rotations per second cruise
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        MotionMagicConfigs motionMagicL = elevatorLowerConfig.MotionMagic;
        motionMagicL.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // (meachanism) rotations per second cruise
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        Slot0Configs upperSlot0 = elevatorUpperConfig.Slot0;
        upperSlot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        upperSlot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        upperSlot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        upperSlot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        upperSlot0.kI = 0; // No output for integrated error
        upperSlot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

        Slot0Configs lowerSlot0 = elevatorLowerConfig.Slot0;
        lowerSlot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
        lowerSlot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        lowerSlot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        lowerSlot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        lowerSlot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        lowerSlot0.kI = 0; // No output for integrated error

        StatusCode statusU = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusU = stage2motor.getConfigurator().apply(elevatorUpperConfig);
            if (statusU.isOK())
                break;
        }
        if (!statusU.isOK()) {
            System.out.println("Could not configure device. Error: " + statusU.toString());
        }

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
        SmartDashboard.putNumber("Bottom Vol", stage1motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Pos", stage1motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Top Vol", stage2motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Top Pos", stage2motor.getPosition().getValueAsDouble());
        // SmartDashboard.putBoolean("Climbing", enabledClimb);

        if (!elevatorBottomSwich.get()) {
            stage1motor.setPosition(0);

        }
        if (!elevatorTopSwich.get()) {
            stage2motor.setPosition(10);

        }

        SmartDashboard.putBoolean("bottomSwitch", getBottomSwitch());
        SmartDashboard.putBoolean("topSwitch", getTopSwitch());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public boolean getBottomSwitch() {
        return !elevatorBottomSwich.get();
    }

    public boolean getTopSwitch() {
        return !elevatorTopSwich.get();
    }

    public boolean getmotor1pos() {
        return stage1motor.getPosition().getValueAsDouble() == 0;
    }

    public boolean getmotor2pos() {
        return stage2motor.getPosition().getValueAsDouble() == 0;
    }

    public boolean isMotorOneAtPos() {
        return Math.abs(stage1motor.getPosition().getValueAsDouble() - stage1motorGoalPos) < .1;
    }

    public boolean isMotorTwoAtPos() {
        return Math.abs(stage2motor.getPosition().getValueAsDouble() - stage2motorGoalPos) < .1;
    }

    public void setFirst() {
        stage1motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.lowerFirst).withSlot(0));
        stage2motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.upperFirst).withSlot(0));
        stage1motorGoalPos = Constants.ElevatorConstants.Positions.lowerFirst;
        stage2motorGoalPos = Constants.ElevatorConstants.Positions.upperFirst;
    }

    public void setSecond() {
        stage1motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.lowerSecond).withSlot(0));
        stage2motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.upperSecond).withSlot(0));
        stage1motorGoalPos = Constants.ElevatorConstants.Positions.lowerSecond;
        stage2motorGoalPos = Constants.ElevatorConstants.Positions.upperSecond;
    }

    public void setThird() {
        stage1motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.lowerThird).withSlot(0));
        stage2motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.upperThird).withSlot(0));
        stage1motorGoalPos = Constants.ElevatorConstants.Positions.lowerThird;
        stage2motorGoalPos = Constants.ElevatorConstants.Positions.upperThird;
    }

    public void setFourth() {
        stage1motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.lowerFourth).withSlot(0));
        stage2motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.upperFourth).withSlot(0));
        stage1motorGoalPos = Constants.ElevatorConstants.Positions.lowerFourth;
        stage2motorGoalPos = Constants.ElevatorConstants.Positions.upperFourth;
    }

    public void setClimb() {
        if (Robot.getInstance().elevatorTestControl.getRightTriggerAxis() >= .5) {
            stage1motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.lowerClimb).withSlot(0));
            stage2motor.setControl(m_motionMagicReqU.withPosition(Constants.ElevatorConstants.Positions.upperClimb).withSlot(0));
            // enabledClimb = true;
            stage1motorGoalPos = Constants.ElevatorConstants.Positions.lowerClimb;
            stage2motorGoalPos = Constants.ElevatorConstants.Positions.upperClimb;
        }
    }

    public void stopBothMotors() {
        stage1motor.set(0);
        stage2motor.set(0);
    }

}
