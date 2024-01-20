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
  // There is a different system used than previous years because MotorControlGroup is deprecated :(.
  // We set a motor to a leader, and make followers follow the leader in the constructor.  
  // Front wheels are leaders for no reason because its redundant
  private CANSparkMax LeftLeader = new CANSparkMax(MotorConstants.frontLeft, MotorType.kBrushless);
  private CANSparkMax RightLeader = new CANSparkMax(MotorConstants.frontRight, MotorType.kBrushless);
  private CANSparkMax LeftFollower = new CANSparkMax(MotorConstants.backLeft, MotorType.kBrushless);
  private CANSparkMax RightFollower = new CANSparkMax(MotorConstants.backRight, MotorType.kBrushless);

  public void setLeftMotors (double volt) {
    LeftLeader.set(volt);
  }
  public void setRightMotors (double volt) {
    RightLeader.set(volt);
  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    LeftFollower.follow(LeftLeader);
    RightFollower.follow(RightLeader);

    LeftLeader.setInverted(true);
    LeftFollower.setInverted(true);
    RightLeader.setInverted(false);
    RightFollower.setInverted(false);
  }

  // runs the motors
  public void setMotors(double left, double right) {
    setLeftMotors(left);
    setRightMotors(right);
  }

  public void setMotorVoltage() {
    setLeftMotorsVoltage(0);
    setRightMotorsVoltage(0);
  }

  public void setLeftMotorsVoltage(double voltage) {
    LeftLeader.setVoltage(voltage);
  }

  public void setRightMotorsVoltage(double voltage) {
    RightLeader.setVoltage(voltage);
  }

  public DifferentialDrive m_drive = new DifferentialDrive(LeftLeader, RightLeader);

  public void arcadeDrive (double speed, double rotation){
    m_drive.arcadeDrive(speed, rotation);
  }

  public void curvatureDrive (double speed, double rotation, boolean allowTurnInPlace){
    m_drive.curvatureDrive(speed, rotation, allowTurnInPlace);
  }

  public DifferentialDrive m_drive = new DifferentialDrive(LeftLeader, RightLeader);

  public void arcadeDrive (double speed, double rotation){
    m_drive.arcadeDrive(speed, rotation);
  }

  public void curvatureDrive (double speed, double rotation, boolean allowTurnInPlace){
    m_drive.curvatureDrive(speed, rotation, allowTurnInPlace);
  }

  @Override
  public void periodic() {
    
  }

  public WPI_PigeonIMU getGyro() {
    return gyro;
  }

  public RelativeEncoder getRightEncoder() {
    return frontRightMotor.getEncoder();
  }


  public RelativeEncoder getLeftEncoder() {
    return frontLeftMotor.getEncoder();
  }
  
}
