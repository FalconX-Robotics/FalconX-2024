// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDControllerSim;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.MotorConstants;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.DashboardHelper;

public class Shooter extends SubsystemBase {
  CANSparkMax shooterLeaderSparkMax = new CANSparkMax(MotorConstants.shooter, MotorType.kBrushless);
  CANSparkMax shooterFollowerSparkMax = new CANSparkMax(MotorConstants.shooterFollower, MotorType.kBrushless);

  DataLog log = DataLogManager.getLog();

  DoubleLogEntry shooterEncoderPositionEntry = new DoubleLogEntry(log, "/shooter/shooter_position");
  DoubleLogEntry shooterEncoderVelocityEntry = new DoubleLogEntry(log, "/shooter/shooter_velocity");

  public void setShooterSparks(double volt){
    shooterLeaderSparkMax.set(volt);
  }

  public SparkPIDController getShooterPidController () {
    if (Robot.isSimulation()) {
      return m_pidControllerSim;
    } else {
      return shooterLeaderSparkMax.getPIDController();
    }
  }

  public double getShooterEncoderVelocity () {
    return shooterLeaderSparkMax.getEncoder().getVelocity();
  }

  /** Creates a new Shooter. */

  public Shooter() {
    // shooterLeaderSparkMax.restoreFactoryDefaults();
    // shooterFollowerSparkMax.restoreFactoryDefaults();
    // armSparkMax.restoreFactoryDefaults();
    // armFollowerSparkMax.restoreFactoryDefaults();

    shooterFollowerSparkMax.follow(shooterLeaderSparkMax, true);

    shooterLeaderSparkMax.getEncoder().setAverageDepth(4);
    shooterLeaderSparkMax.getEncoder().setMeasurementPeriod(8);

    shooterLeaderSparkMax.setSmartCurrentLimit(50);
    shooterLeaderSparkMax.setIdleMode(IdleMode.kCoast);
    shooterLeaderSparkMax.setInverted(false);
    shooterLeaderSparkMax.burnFlash();

    shooterFollowerSparkMax.setSmartCurrentLimit(50);
    shooterFollowerSparkMax.setIdleMode(IdleMode.kCoast);
    shooterFollowerSparkMax.setInverted(true);
    shooterFollowerSparkMax.burnFlash();
    
    if (Robot.isSimulation()) {
      m_pidControllerSim = new SparkPIDControllerSim(shooterLeaderSparkMax);
    }
  }

  public void setShooterReference(double setPoint) {
    getShooterPidController().setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  // public boolean armJoystickActive () {
  //   return Math.abs(m_settings.noteController.getArmJoystickValue()) > 0;
  // }

  public boolean velocityIsWithinTarget (double target, double leniency) {
    return (getShooterEncoderVelocity() >= target - leniency
         && getShooterEncoderVelocity() <= target + leniency);
  }

  public boolean velocityIsWithinTarget () {
    return (getShooterEncoderVelocity() >= 2450. - 35.
         && getShooterEncoderVelocity() <= 2450. + 35.);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder Position", shooterLeaderSparkMax.getEncoder().getPosition());
    SmartDashboard.putNumber("Shooter Encoder Velocity", shooterLeaderSparkMax.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Shooter Velocity Within Target Speed", velocityIsWithinTarget(2450., 50.));

    shooterEncoderPositionEntry.append(shooterLeaderSparkMax.getEncoder().getPosition());
    shooterEncoderVelocityEntry.append(shooterLeaderSparkMax.getEncoder().getVelocity());
  }

  // Moment of inertia for uniform cylinder = 1/2 * m * r^2.
  FlywheelSim shooterSim = new FlywheelSim(DCMotor.getNEO(2), 1, 
      0.5 * BaseUnits.Mass.convertFrom(8, Units.Pounds) *
      Math.pow(BaseUnits.Distance.convertFrom(2, Units.Inches), 2));
  SparkPIDControllerSim m_pidControllerSim;

  @Override
  public void simulationPeriodic() {
    SparkPIDControllerSim pidSim = (SparkPIDControllerSim) getShooterPidController();
    // Simulate every 1ms to match PID controller implementation on Spark MAX
    for (int i = 0; i < 20; i++) {
      // Set the input to the physics simulation to the output of the simulated PID controller
      shooterSim.setInputVoltage(pidSim.getVoltageOutput());
      shooterSim.update(0.001);

      // Update the velocity input to the PID controller using the physics sim
      pidSim.setVelocity(shooterSim.getAngularVelocityRPM());
      pidSim.update(0.001);
    }
    
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Sim Shooter Voltage", pidSim.getVoltageOutput());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Sim Shooter RPM", shooterSim.getAngularVelocityRPM());
  }
}
