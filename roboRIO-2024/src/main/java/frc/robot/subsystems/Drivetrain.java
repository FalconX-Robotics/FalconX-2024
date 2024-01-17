// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontLeftMotor = new CANSparkMax(MotorConstants.frontLeft, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(MotorConstants.frontRight, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(MotorConstants.backLeft, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(MotorConstants.backRight, MotorType.kBrushless);

  private PigeonIMU gyro = new PigeonIMU(Constants.PIGEON_PORT);

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
    
    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);
    frontRightMotor.setInverted(false);
    backRightMotor.setInverted(false);
  }

  // runs the motors
  public void setMotors(double leftVolt, double rightVolt) {
    setLeftMotors(leftVolt);
    setRightMotors(rightVolt);
  }

  @Override
  public void periodic() {
    
  }

  public PigeonIMU getGyro() {
    return gyro;
  }

  public RelativeEncoder getRightEncoder() {
    return frontRightMotor.getEncoder();
  }


  public RelativeEncoder getLeftEncoder() {
    return frontLeftMotor.getEncoder();
  }
  
}
