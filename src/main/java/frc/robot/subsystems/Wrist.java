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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
public class Wrist extends SubsystemBase {

    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private SparkMax wristMotor;
    private DigitalInput rotationZero;
    public double wristTarget;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
    *
    */
    public Wrist() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        wristMotor = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig NewSparkMaxConfig = new SparkMaxConfig();
        wristMotor.configure(NewSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        NewSparkMaxConfig.inverted(false);
        NewSparkMaxConfig.idleMode(IdleMode.kBrake);
        NewSparkMaxConfig.smartCurrentLimit(10, 10);

        rotationZero = new DigitalInput(4);
        addChild("rotationZero", rotationZero);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        closedLoopController = wristMotor.getClosedLoopController();
        encoder = wristMotor.getEncoder();
        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        NewSparkMaxConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);

        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        NewSparkMaxConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed
                // loop slot, as it will default to slot 0.
                .p(0.4)
                .i(0)
                .d(0)
                .outputRange(-1, 1);

        NewSparkMaxConfig.closedLoop.maxMotion
                // Set MAXMotion parameters for position control. We don't need to pass
                // a closed loop slot, as it will default to slot 0.
                .maxVelocity(8000)//11000
                .maxAcceleration(5000)
                .allowedClosedLoopError(Constants.WristConstants.tolerance);

        /*
         * Apply the configuration to the SPARK MAX.
         *
         * kResetSafeParameters is used to get the SPARK MAX to a known state. This
         * is useful in case the SPARK MAX is replaced.
         *
         * kPersistParameters is used to ensure the configuration is not lost when
         * the SPARK MAX loses power. This is useful for power cycles that may occur
         * mid-operation.
         */
        wristMotor.configure(NewSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Target Position", 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (switchValue() == true) {
            encoder.setPosition(0);
        }

        SmartDashboard.putBoolean("switchWrist", switchValue());
        SmartDashboard.putNumber("wrist pos", encoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void rotateToPosition() {

        closedLoopController.setReference(wristTarget,
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0);

    }

    public boolean atPosition() {
        return Math.abs(encoder.getPosition() - wristTarget) < Constants.WristConstants.tolerance; // degree tolerance
    }// + or - 1 degree on the output shaft

    public boolean switchValue() {
        return Robot.getInstance().getWristTripped();
    }

    public void stopMotor() {
        wristMotor.set(0);
    }

    public void runInTheZeroWay(){
        wristMotor.set(-.2);
    }

}