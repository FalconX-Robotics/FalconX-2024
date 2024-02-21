// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
  CANSparkMax shooterArmSparkMax = new CANSparkMax(MotorConstants.shooterArm, MotorType.kBrushless);
  CANSparkMax shooterSparkMax = new CANSparkMax(MotorConstants.shooter, MotorType.kBrushless);
  
  DataLog log = DataLogManager.getLog();

  DoubleLogEntry shooterArmEncoderPositionEntry = new DoubleLogEntry(log, "/shooter/shooter_arm_position");
  DoubleLogEntry shooterEncoderPositionEntry = new DoubleLogEntry(log, "/shooter/shooter_position");
  DoubleLogEntry shooterArmEncoderVelocityEntry = new DoubleLogEntry(log, "/shooter/shooter_arm_velocity");
  DoubleLogEntry shooterEncoderVelocityEntry = new DoubleLogEntry(log, "/shooter/shooter_velocity");

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
    
    SmartDashboard.putNumber("Shooter Arm Encoder Position", shooterArmSparkMax.getEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Encoder Position", shooterSparkMax.getEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Arm Encoder Velocity", shooterArmSparkMax.getEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Encoder Velocity", shooterSparkMax.getEncoder.getVelocity());

    shooterArmEncoderPositionEntry.append(shooterArmSparkMax.getEncoder.getPosition());
    shooterEncoderPositionEntry.append(shooterSparkMax.getEncoder.getPosition());
    shooterArmEncoderVelocityEntry.append(shooterArmSparkMax.getEncoder.getVelocity());
    shooterEncoderVelocityEntry.append(shooterSparkMax.getEncoder.getVelocity());
  }
}
