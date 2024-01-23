// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax shooterArmSparkMax = new CANSparkMax(MotorConstants.shooterArm, MotorType.kBrushless);
  CANSparkMax shooterSparkMax = new CANSparkMax(MotorConstants.shooter, MotorType.kBrushless);  

  public void setArmSpark(double volt){
    shooterArmSparkMax.set(volt);
  }

  public void setShooterSpark(double volt){
    shooterSparkMax.set(volt);
  }

  public SparkPIDController getShooterPidController () {
    return shooterSparkMax.getPIDController();
  }

  public double getShooterArmEncoderRotation() {
    return shooterArmSparkMax.getEncoder().getPosition();
  }

  /** Creates a new Shooter. */
  public Shooter() {
    //TODO: Set inverted if applicable
  }

  @Override
  public void periodic() {
    shooterArmSparkMax.getEncoder().setPositionConversionFactor(1.);
    shooterArmSparkMax.getEncoder().setPosition(0); // resets position of encoder

    // TODO: set position conversion factor if necessary
    // This method will be called once per scheduler run
  }
}
