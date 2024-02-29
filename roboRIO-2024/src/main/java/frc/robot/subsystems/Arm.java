// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.ArmFeedForwardValues;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RatioConstants;

public class Arm extends SubsystemBase {
  ArmFeedforward armFeedforward;
  CANSparkMax armSparkMax = new CANSparkMax(MotorConstants.arm, MotorType.kBrushless);
  CANSparkMax armFollowerSparkMax = new CANSparkMax(MotorConstants.armFollower, MotorType.kBrushless);
  Settings m_settings;
  DataLog log = DataLogManager.getLog();
  DoubleLogEntry shooterArmEncoderVelocityEntry = new DoubleLogEntry(log, "/arm/shooter_arm_velocity");
  DoubleLogEntry shooterArmEncoderPositionEntry = new DoubleLogEntry(log, "/arm/shooter_arm_position");
  double goalPositionRad = 0;
  TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ArmFeedForwardValues.maxVelocity, ArmFeedForwardValues.maxAccelasfklj));

  /** Creates a new Arm. */
  public Arm(Settings settings) {
    m_settings = settings;
    armFeedforward = new ArmFeedforward(
      ArmFeedForwardValues.staticGain,
      ArmFeedForwardValues.gravityGain,
      ArmFeedForwardValues.velocityGain
    );

    armFollowerSparkMax.follow(armSparkMax, true);
    
    armFollowerSparkMax.setIdleMode(IdleMode.kBrake);
    armFollowerSparkMax.getEncoder().setPositionConversionFactor(1/RatioConstants.ArmGearRatio/(2*Math.PI));
    armFollowerSparkMax.getEncoder().setPosition(0);
    armFollowerSparkMax.setInverted(true);
    armFollowerSparkMax.burnFlash();

    armSparkMax.setIdleMode(IdleMode.kBrake);
    armSparkMax.getEncoder().setPositionConversionFactor(1/RatioConstants.ArmGearRatio/(2*Math.PI));
    armSparkMax.getEncoder().setPosition(0);
    armSparkMax.setInverted(false);
    armSparkMax.burnFlash();
  }

  public void setSparks (double percentOutput) {
    armSparkMax.set(percentOutput);
  }

  public void setSparksVoltage(double volt){
    armSparkMax.setVoltage(volt);
  }

  public double getArmEncoderRotation() {
    return armSparkMax.getEncoder().getPosition();
  }
  public double getArmEncoderVelocity() {
    return armSparkMax.getEncoder().getVelocity();
  }

  public void setGoalPositionRadians(double position) {
    goalPositionRad = position;
  }
  
  public void goToGoalPosition() {
    TrapezoidProfile.State targetState = trapezoidProfile.calculate(
      .02, 
      new TrapezoidProfile.State(getArmEncoderRotation(), getArmEncoderVelocity()),
      new TrapezoidProfile.State(goalPositionRad, 0)
    );
    double voltageOutput = armFeedforward.calculate(targetState.position, targetState.velocity);
    setSparksVoltage(MathUtil.clamp(voltageOutput, -12., 12.));
  }

  @Override
  public void periodic() {
    // If no current command, set arm via joystick value.
    // if(this.getCurrentCommand() == null) {
    //   if (armJoystickActive()) {
    //     // armSparkMax.set(limitArmViaEncoder(m_settings.noteController.getArmJoystickValue())); return;
    //   }
      //TODO make work
      // armSparkMax.set(feedforward());
      armSparkMax.set(m_settings.noteController.getArmJoystickValue() * .3);
      // }
      
      goToGoalPosition();
    
    SmartDashboard.putNumber("Shooter Arm Encoder Position", armSparkMax.getEncoder().getPosition());
    SmartDashboard.putNumber("Shooter Arm Encoder Velocity", armSparkMax.getEncoder().getVelocity());
    shooterArmEncoderPositionEntry.append(armSparkMax.getEncoder().getPosition());
    shooterArmEncoderVelocityEntry.append(armSparkMax.getEncoder().getVelocity());
  }
}
