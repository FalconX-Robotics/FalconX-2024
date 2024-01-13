// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public DifferentialDrive arcadeDrive = new DifferentialDrive(LeftLeader, RightLeader);

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
  public void setMotors(double leftVolt, double rightVolt) {
    setLeftMotors(leftVolt);
    setRightMotors(rightVolt);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
