// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Drivetrain extends SubsystemBase {
  CANSparkMax frontLeftMotor = new CANSparkMax(MotorConstants.frontLeft, MotorType.kBrushless);
  CANSparkMax frontRightMotor = new CANSparkMax(MotorConstants.frontRight, MotorType.kBrushless);
  CANSparkMax backLeftMotor = new CANSparkMax(MotorConstants.backLeft, MotorType.kBrushless);
  CANSparkMax backRightMotor = new CANSparkMax(MotorConstants.backRight, MotorType.kBrushless);
  
  // DEPRECATED??
  // MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  
  public void setLeftMotors (double volt) {
    frontLeftMotor.set(volt);
    backLeftMotor.set(volt);
  }
  public void setRightMotors (double volt) {
    frontRightMotor.set(volt);
    backRightMotor.set(volt);
  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {

  }
  // runs the motors
  public void setMotors(double leftVolt, double rightVolt) {
    setLeftMotors(leftVolt);
    setRightMotors(rightVolt);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
