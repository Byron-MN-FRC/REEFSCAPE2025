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

public class Algae extends SubsystemBase {
  /** Creates a new Algae. */

  private SparkMax algaeClawMotor;

  public Algae() {

    algaeClawMotor = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig AlgaeSparkMaxConfig = new SparkMaxConfig();
    AlgaeSparkMaxConfig.inverted(true);
    AlgaeSparkMaxConfig.idleMode(IdleMode.kBrake);
    AlgaeSparkMaxConfig.smartCurrentLimit(5, 10);
    algaeClawMotor.configure(AlgaeSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Algae", Robot.getInstance().getAlgaeDetect());
  }

  public void algaeRotateInwards() {
    algaeClawMotor.set(1);
  }

  public void algaeRotateOutwards() {
    algaeClawMotor.set(-1);
  }

  public void algaeZero() {
    algaeClawMotor.set(0);
  }

 

}
