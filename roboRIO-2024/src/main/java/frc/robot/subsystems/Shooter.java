// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDControllerSim;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RatioConstants;
import frc.robot.Settings.FeedForwardValues;

public class Shooter extends SubsystemBase {
  CANSparkMax armSparkMax = new CANSparkMax(MotorConstants.arm, MotorType.kBrushless);
  CANSparkMax armFollowerSparkMax = new CANSparkMax(MotorConstants.armFollower, MotorType.kBrushless);
  
  CANSparkMax shooterSparkMax = new CANSparkMax(MotorConstants.shooter, MotorType.kBrushless);
  CANSparkMax shooterFollowerSparkMax = new CANSparkMax(MotorConstants.shooterFollower, MotorType.kBrushless);

  Settings m_settings;

  ArmFeedforward armFeedforward = new ArmFeedforward(
    m_settings.feedForwardValues.staticGain,
    m_settings.feedForwardValues.gravityGain,
    m_settings.feedForwardValues.velocityGain
  );

  public void setArmSpark(double volt){
    armSparkMax.set(volt);
  }

  public void setShooterSpark(double volt){
    shooterSparkMax.set(volt);
  }

  public SparkPIDController getShooterPidController () {
    if (Robot.isSimulation()) {
      return m_pidControllerSim;
    } else {
      return shooterSparkMax.getPIDController();
    }
  }

  public double getShooterArmEncoderRotation() {
    return armSparkMax.getEncoder().getPosition();
  }

  /** Creates a new Shooter. */

  public Shooter(Settings settings) {
    m_settings = settings;
    armFeedforward = new ArmFeedforward(
      m_settings.feedForwardValues.staticGain,
      m_settings.feedForwardValues.gravityGain,
      m_settings.feedForwardValues.velocityGain
    );
    shooterSparkMax.restoreFactoryDefaults();
    shooterFollowerSparkMax.restoreFactoryDefaults();
    armSparkMax.restoreFactoryDefaults();
    armFollowerSparkMax.restoreFactoryDefaults();

    shooterFollowerSparkMax.follow(shooterSparkMax, true);
    armFollowerSparkMax.follow(armSparkMax, true);
    
    armFollowerSparkMax.setIdleMode(IdleMode.kBrake);
    armFollowerSparkMax.getEncoder().setPositionConversionFactor(RatioConstants.ArmGearRatio);
    armFollowerSparkMax.getEncoder().setPosition(0);
    armFollowerSparkMax.setInverted(true);
    armFollowerSparkMax.burnFlash();

    armSparkMax.setIdleMode(IdleMode.kBrake);
    armSparkMax.getEncoder().setPositionConversionFactor(RatioConstants.ArmGearRatio);
    armSparkMax.getEncoder().setPosition(0);
    armSparkMax.setInverted(false);
    armSparkMax.burnFlash();

    shooterSparkMax.setIdleMode(IdleMode.kCoast);
    shooterSparkMax.setInverted(false);
    shooterSparkMax.burnFlash();

    shooterFollowerSparkMax.setIdleMode(IdleMode.kCoast);
    shooterFollowerSparkMax.setInverted(true);
    shooterFollowerSparkMax.burnFlash();

    // TODO: Change position conversion factor as needed
    
    if (Robot.isSimulation()) {
      m_pidControllerSim = new SparkPIDControllerSim(shooterSparkMax);
    }
  }

  /**
   * A method to block the arm from moving too far.
   * @param input A double from -1 to 1. Put joystick value here
   * @return The same double, unless moved too far.
   */
  private double limitArmViaEncoder (double input){
    // TODO: set the position temporary values under this line.
    if (armSparkMax.getEncoder().getPosition() >= 1000. && input >= 0.) {
      return 0.;
    }
    if (armSparkMax.getEncoder().getPosition() <= -1000. && input <= 0.) {
      return 0.;
    }
    return input;
  }

  private double feedforward () {
    return armFeedforward.calculate(Math.toDegrees(getShooterArmEncoderRotation()), 0, 0);
  }

  public void setShooterReference(double setPoint) {
    getShooterPidController().setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setMotors(double speed){
    shooterSparkMax.set(speed);
    shooterFollowerSparkMax.set(speed);

  }

  public boolean armJoystickActive () {
    return Math.abs(m_settings.noteController.getArmJoystickValue()) > 0;
  }

  @Override
  public void periodic() {
    // If no current command, set arm via joystick value.
    if(this.getCurrentCommand() == null) {
      if (armJoystickActive()) {
        armSparkMax.set(limitArmViaEncoder(m_settings.noteController.getArmJoystickValue())); return;
      }
      //TODO does this work lol
      armSparkMax.set(feedforward());
    }
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
    
    SmartDashboard.putNumber("Sim Shooter Voltage", pidSim.getVoltageOutput());
    SmartDashboard.putNumber("Sim Shooter RPM", shooterSim.getAngularVelocityRPM());
  }
}
