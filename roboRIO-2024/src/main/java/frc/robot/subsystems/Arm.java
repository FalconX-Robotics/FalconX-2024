// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardHelper;
import frc.robot.Robot;
import frc.robot.Constants.ArmFeedForwardConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RatioConstants;
import frc.robot.simulation.RelativeEncoderSim;
import frc.robot.Settings;

public class Arm extends SubsystemBase {
  CANSparkMax armSparkMax = new CANSparkMax(MotorConstants.arm, MotorType.kBrushless);
  CANSparkMax armFollowerSparkMax = new CANSparkMax(MotorConstants.armFollower, MotorType.kBrushless);
  DataLog log = DataLogManager.getLog();
  DoubleLogEntry shooterArmEncoderVelocityEntry = new DoubleLogEntry(log, "/arm/shooter_arm_velocity");
  DoubleLogEntry shooterArmEncoderPositionEntry = new DoubleLogEntry(log, "/arm/shooter_arm_position");
  Settings m_settings;
  RelativeEncoder m_armEncoder;

  /** Creates a new Arm. */
  public Arm(Settings settings) {
    m_settings = settings;
    armFollowerSparkMax.follow(armSparkMax, true);
    
    armFollowerSparkMax.setIdleMode(IdleMode.kBrake);
    armFollowerSparkMax.getEncoder().setPositionConversionFactor((2*Math.PI)/(RatioConstants.ArmGearRatio));
    armFollowerSparkMax.getEncoder().setPosition(0.);
    armFollowerSparkMax.setSmartCurrentLimit(40);
    armFollowerSparkMax.burnFlash();

    if (Robot.isSimulation()) {
      m_armEncoder = new RelativeEncoderSim();
    } else {
      m_armEncoder = armSparkMax.getEncoder();
      m_armEncoder.setPositionConversionFactor((2*Math.PI)/RatioConstants.ArmGearRatio);
    }

    armSparkMax.setIdleMode(IdleMode.kBrake);
    // m_armEncoder.setPositionConversionFactor(1/(*(2*Math.PI)RatioConstants.ArmGearRatio));
    m_armEncoder.setPosition(0.);
    armSparkMax.setInverted(false);
    armSparkMax.setSmartCurrentLimit(40);
    armSparkMax.burnFlash();

    SmartDashboard.putNumber("arm max speed", .3);
  }

  public boolean isStored(){
    return getRotation() <= Math.toRadians(5);
  }
  public boolean isAtMaxExtension(){
    return getRotation() >= Math.toRadians(120);
  }

  public void setSparks (double percentOutput) {
    if(isStored()){
      armSparkMax.set(MathUtil.clamp(percentOutput, 0., 1.5));
      return;
    }
    if(isAtMaxExtension()){
      armSparkMax.set(MathUtil.clamp(percentOutput, -1.5, 0));
      return;
    }
    armSparkMax.set(percentOutput);
  }

  public void setSparksVoltage(double volt){
    if (isStored()){
      armSparkMax.setVoltage(MathUtil.clamp(volt, 0., 15.));
    }
    if(isAtMaxExtension()){
      armSparkMax.setVoltage(MathUtil.clamp(volt, -15., 0.));
      return;
    }
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
    //     // armSparkMax.set(limitArmViaEncoder(m_settings.noteSettings.getManualArmJoystickValue())); return;
    //   }
      //TODO make work
      // armSparkMax.set(feedforward());
      // }
      // if (Math.abs(m_settings.noteSettings.getManualArmJoystickValue()) > 0.2){
      // double armAppliedOutput = m_settings.noteSettings.getManualArmJoystickValue()
      //   * SmartDashboard.getNumber("arm max speed", .3) * 12;
      // armSparkMax.setVoltage(armAppliedOutput);
      // SmartDashboard.putNumber("Arm Applied Output", armAppliedOutput);
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
    Math.pow(BaseUnits.Distance.convertFrom(18, Units.Inches), 2) * 
      BaseUnits.Mass.convertFrom(20, Units.Pounds) / 3,
    BaseUnits.Distance.convertFrom(18, Units.Inches),
    BaseUnits.Angle.convertFrom(0, Units.Degrees),
    BaseUnits.Angle.convertFrom(270, Units.Degrees),
    true,
    BaseUnits.Angle.convertFrom(0, Units.Degrees)
  );


  @Override
  public void simulationPeriodic() {
    double voltage = 0;
    // account for weird get/getAppliedOutput behavior (get only works with set)
    // getAppliedOutput only works with setVoltage()
    double appliedOutputVoltage = armSparkMax.getAppliedOutput();
    double getVoltage = armSparkMax.get() * RobotController.getBatteryVoltage();
    if (Math.abs(appliedOutputVoltage) > 0.0001) {
      voltage = appliedOutputVoltage;
    } else if (Math.abs(getVoltage) > 0.0001) {
      voltage = getVoltage;
    }
    m_armSim.setInputVoltage(voltage);
    m_armSim.update(0.02);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "Arm Sim Applied Output", voltage);
    
    RelativeEncoderSim armEncoderSim = (RelativeEncoderSim) m_armEncoder;
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "Arm Angle Radians", 
      m_armSim.getAngleRads());

    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "Arm Velocity Radians/Sec", 
      m_armSim.getVelocityRadPerSec());

    armEncoderSim.setSimulationPosition(m_armSim.getAngleRads());
    armEncoderSim.setSimulationVelocity(m_armSim.getVelocityRadPerSec());
  }
}
