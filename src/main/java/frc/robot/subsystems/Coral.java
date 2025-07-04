// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Coral extends SubsystemBase {
  /** Creates a new Claw. */

  private SparkMax coralClawMotor;
  public double coralDropSpeed;

  public Coral() {

    coralClawMotor = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig CoralSparkMaxConfig = new SparkMaxConfig();
    CoralSparkMaxConfig.inverted(true);
    CoralSparkMaxConfig.idleMode(IdleMode.kBrake);
    CoralSparkMaxConfig.smartCurrentLimit(15, 10);
    coralClawMotor.configure(CoralSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral", Robot.getInstance().getCoralDetect());
    // SmartDashboard.putBoolean("Algae", getAlgaeDetect());
  }

  // public Boolean getCoralDetect() {
  //   return clawCandi.getS1Closed().getValue();
  // }

  public void coralRotateInwards() {
    coralClawMotor.set(.5);
  }

  public void coralRotateOutwards() {
    coralClawMotor.set(coralDropSpeed);
  }

  public void coralZero() {
    coralClawMotor.set(0);
  }

}
