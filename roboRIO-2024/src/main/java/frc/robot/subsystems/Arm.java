// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardHelper;
import frc.robot.Robot;
import frc.robot.Constants.ArmFeedForwardConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RatioConstants;
import frc.robot.simulation.RelativeEncoderSim;

public class Arm extends SubsystemBase {
  CANSparkMax armSparkMax = new CANSparkMax(MotorConstants.arm, MotorType.kBrushless);
  CANSparkMax armFollowerSparkMax = new CANSparkMax(MotorConstants.armFollower, MotorType.kBrushless);
  DataLog log = DataLogManager.getLog();
  DoubleLogEntry shooterArmEncoderVelocityEntry = new DoubleLogEntry(log, "/arm/shooter_arm_velocity");
  DoubleLogEntry shooterArmEncoderPositionEntry = new DoubleLogEntry(log, "/arm/shooter_arm_position");

  RelativeEncoder m_armEncoder;

  /** Creates a new Arm. */
  public Arm() {
    armFollowerSparkMax.follow(armSparkMax, true);
    
    armFollowerSparkMax.setIdleMode(IdleMode.kBrake);
    armFollowerSparkMax.getEncoder().setPositionConversionFactor(1/(RatioConstants.ArmGearRatio*(2*Math.PI)));
    armFollowerSparkMax.getEncoder().setPosition(ArmFeedForwardConstants.offset);
    armFollowerSparkMax.setInverted(true);
    armFollowerSparkMax.burnFlash();

    if (Robot.isSimulation()) {
      m_armEncoder = new RelativeEncoderSim();
    } else {
      m_armEncoder = armSparkMax.getEncoder();
      m_armEncoder.setPositionConversionFactor(1/RatioConstants.ArmGearRatio/(2*Math.PI));
    }

    armSparkMax.setIdleMode(IdleMode.kBrake);
    armSparkMax.getEncoder().setPositionConversionFactor(1/(RatioConstants.ArmGearRatio*(2*Math.PI)));
    armSparkMax.getEncoder().setPosition(ArmFeedForwardConstants.offset);
    armSparkMax.setInverted(false);
    armSparkMax.burnFlash();

  }

  public void setSparks (double percentOutput) {
    armSparkMax.set(percentOutput);
  }

  public void setSparksVoltage(double volt){
    armSparkMax.setVoltage(volt);
  }

  public double getRotation() {
    return m_armEncoder.getPosition();
  }
  public double getVelocity() {
    return m_armEncoder.getVelocity();
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
      // }
      // if (Math.abs(m_settings.noteController.getArmJoystickValue()) > 0.2){
      //   armSparkMax.set(m_settings.noteController.getArmJoystickValue() * .3);
      // }
    
    SmartDashboard.putNumber("Shooter Arm Encoder Position", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Arm Encoder Velocity", m_armEncoder.getVelocity());
    shooterArmEncoderPositionEntry.append(m_armEncoder.getPosition());
    shooterArmEncoderVelocityEntry.append(m_armEncoder.getVelocity());
  }

  private SingleJointedArmSim m_armSim = new SingleJointedArmSim(
    DCMotor.getNEO(2),
    RatioConstants.ArmGearRatio,
    // MOI = mr^2 for a point
    Math.pow(BaseUnits.Distance.convertFrom(24, Units.Inches), 2) * 
      BaseUnits.Mass.convertFrom(20, Units.Pounds),
    BaseUnits.Distance.convertFrom(24, Units.Inches),
    BaseUnits.Angle.convertFrom(70, Units.Degrees),
    BaseUnits.Angle.convertFrom(270, Units.Degrees),
    true,
    BaseUnits.Angle.convertFrom(70, Units.Degrees)
  );


  @Override
  public void simulationPeriodic() {
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Arm Applied Output", armSparkMax.getAppliedOutput());
    m_armSim.setInputVoltage(armSparkMax.getAppliedOutput());
    m_armSim.update(0.02);

    
    RelativeEncoderSim armEncoderSim = (RelativeEncoderSim) m_armEncoder;
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "Arm Angle Radians", 
      m_armSim.getAngleRads());

    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "Arm Velocity Radians/Sec", 
      m_armSim.getVelocityRadPerSec());

    armEncoderSim.setSimulationPosition(m_armSim.getAngleRads());
    armEncoderSim.setSimulationVelocity(m_armSim.getVelocityRadPerSec());
  }
}
