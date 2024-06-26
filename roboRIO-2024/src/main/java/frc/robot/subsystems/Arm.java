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
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DashboardHelper;
import frc.robot.Robot;
import frc.robot.Constants.ArmFeedForwardConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RatioConstants;
import frc.robot.DashboardHelper.LogLevel;
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
    armFollowerSparkMax.getEncoder().setMeasurementPeriod(8);
    armFollowerSparkMax.getEncoder().setAverageDepth(4);
    armFollowerSparkMax.getEncoder().setPositionConversionFactor((2*Math.PI)/(RatioConstants.ArmGearRatio));
    armFollowerSparkMax.getEncoder().setVelocityConversionFactor((2*Math.PI)/(RatioConstants.ArmGearRatio)/60.);
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
    m_armEncoder.setMeasurementPeriod(8);
    m_armEncoder.setAverageDepth(4);
    m_armEncoder.setPosition(0.);
    m_armEncoder.setVelocityConversionFactor((2*Math.PI)/(RatioConstants.ArmGearRatio)/60.);
    armSparkMax.setInverted(false);
    armSparkMax.setSmartCurrentLimit(40);
    armSparkMax.burnFlash();

    SmartDashboard.putNumber("arm max speed", .3);
  }

  public boolean isStored(){
    return getRotation() <= Math.toRadians(5);
  }
  public boolean isAtMaxExtension(){
    return getRotation() >= Math.toRadians(105);
  }

  public boolean isNearTarget(double targetDegrees, double leniency) {
    return Math.abs(Math.toDegrees(getRotation())-targetDegrees)<leniency;
  }

  public void setSparks (double percentOutput) {
    if(isStored()){
      SmartDashboard.putNumber("Arm Volt Set", MathUtil.clamp(percentOutput, -.5/12, 1.2) * 12);
      armSparkMax.set(MathUtil.clamp(percentOutput, -.5/12, 1.2));
      return;
    }
    if(isAtMaxExtension()){
      SmartDashboard.putNumber("Arm Volt Set", MathUtil.clamp(percentOutput, -1.5, Math.cos(getRotation())*ArmFeedForwardConstants.gravityGain/12.) * 12);
      armSparkMax.set(MathUtil.clamp(percentOutput, -1.5, Math.min(Math.cos(getRotation())*ArmFeedForwardConstants.gravityGain/12., 0)));
      return;
    }
    armSparkMax.set(percentOutput);
    SmartDashboard.putNumber("Arm Volt Set", percentOutput);
  }

  public void setSparksVoltage(double volt){
    if (isStored()){
      SmartDashboard.putNumber("Arm Volt Set", MathUtil.clamp(volt, -.5, 15.));
      armSparkMax.setVoltage(MathUtil.clamp(volt, -.5, 15.));
    }
    if(isAtMaxExtension()){
      // SmartDashboard.putNumber("Arm Volt Set", MathUtil.clamp(volt, -15., -.3));
      // armSparkMax.setVoltage(MathUtil.clamp(volt, -15., -.3));
      SmartDashboard.putNumber("Arm Volt Set", MathUtil.clamp(volt, -15., Math.cos(getRotation())*ArmFeedForwardConstants.gravityGain));
      armSparkMax.setVoltage(MathUtil.clamp(volt, -15., Math.min(Math.cos(getRotation())*ArmFeedForwardConstants.gravityGain, 0)));
      return;
    }
    armSparkMax.setVoltage(volt);
    SmartDashboard.putNumber("Arm Volt Set", volt);
  }

  public void resetEncoder (){
    armSparkMax.getEncoder().setPosition(0.);
  }

  public double getRotation() {
    return m_armEncoder.getPosition();
  }
  public double getVelocity() {
    return m_armEncoder.getVelocity();
  }

  public boolean armJoystickActive () {
    return m_settings.noteSettings.getManualArmJoystickValue() != 0.;
  }

  @Override
  public void periodic() {

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

    // if (getCurrentCommand() == null && m_settings.noteSettings.getManualArmJoystickValue() == 0){
    //   setSparksVoltage(Math.cos(getRotation()) * ArmFeedForwardConstants.gravityGain);;
    // }
  }

  private SingleJointedArmSim m_armSim = new SingleJointedArmSim(
    DCMotor.getNEO(2),
    RatioConstants.ArmGearRatio,
    // MOI = mr^2 for a point
    Math.pow(BaseUnits.Distance.convertFrom(24, Units.Inches), 2) * 
      BaseUnits.Mass.convertFrom(20, Units.Pounds),
    BaseUnits.Distance.convertFrom(24, Units.Inches),
    BaseUnits.Angle.convertFrom(0, Units.Degrees),
    BaseUnits.Angle.convertFrom(270, Units.Degrees),
    true,
    BaseUnits.Angle.convertFrom(0, Units.Degrees)
  );


  @Override
  public void simulationPeriodic() {
    m_armSim.update(0.02);
    double temporaryNumber = armSparkMax.get() * 12;
    m_armSim.setInputVoltage(temporaryNumber);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "Arm Applied Output", temporaryNumber);
    
    RelativeEncoderSim armEncoderSim = (RelativeEncoderSim) m_armEncoder;
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "Arm Angle Radians", 
      m_armSim.getAngleRads());

    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "Arm Velocity Radians/Sec", 
      m_armSim.getVelocityRadPerSec());

    armEncoderSim.setSimulationPosition(m_armSim.getAngleRads());
    armEncoderSim.setSimulationVelocity(m_armSim.getVelocityRadPerSec());
  }

  /**
   * The angle to rotate the arm to, translated based on the initial rotation of the arm.
   * The {@code targetDistance} param is required to account for the distance from the note and camera.
   * @param targetAngle target angle in radians
   * @param targetDistance in meters
   */
  public double targetAngleToArmAngle(double targetAngle, double targetDistance) {
    DashboardHelper.putString(LogLevel.Verbose, "angle and distance", "[" + targetAngle + ", " + targetDistance + "]");
    double angleForCamera = Math.toRadians(Constants.ARM_RESTING_ANGLE) - targetAngle;

    double estimatedDistanceShooterToCamera = Units.Meters.convertFrom(Constants.ARM_ESTIMATED_DISTANCE_TO_CAMERA, Units.Inches);
    double horizontalDistCameraToTarget = Math.cos(angleForCamera) * targetDistance;
    double horizontalDistShooterToTarget = estimatedDistanceShooterToCamera + horizontalDistCameraToTarget;
    double verticalDistCameraToTarget = Math.abs(Math.sin(angleForCamera) * targetDistance);
    double targetDistanceFromShooter = Math.sqrt(Math.pow(verticalDistCameraToTarget+(2*2.54*.01),2)+Math.pow(horizontalDistShooterToTarget,2));
    // double targetDistanceFromShooter = 
    double magicFudgeFactorOfDoomToMakeItAllWork = Math.toRadians(13.5) * (targetDistance/2.6);
    double answer = Math.acos(horizontalDistShooterToTarget/targetDistanceFromShooter) + magicFudgeFactorOfDoomToMakeItAllWork;
    DashboardHelper.putNumber(LogLevel.Debug, "target angle to arm angle", answer);
    return answer;
  }
}
