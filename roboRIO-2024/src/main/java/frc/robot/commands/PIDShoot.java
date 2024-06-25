// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class PIDShoot extends Command {
  private Index m_index;
  private Shooter m_shooter;
  private SparkPIDController m_pidController;
  private double RPMin, leniency, initialTimestamp;

  /** Creates a new Shoot. */
  public PIDShoot(Shooter shooter, Index index) {
    this(shooter, index, 50., 2450.);
    SmartDashboard.putString("index", index.getName());
  }

  public PIDShoot(Shooter shooter, Index index, double leniency, double RPMin) {
    this(shooter, index, 0.001, 0, 0.0001, 0, 0.000178, -1, 1, RPMin);
  }

  public PIDShoot (Shooter shooter, Index index, double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput, double RPMin) {
    m_shooter = shooter;
    m_index = index;
    this.leniency = leniency;
    m_pidController = m_shooter.getShooterPidController();
    addRequirements(shooter);
    setPID(kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput, RPMin);
    SmartDashboard.putNumber("Shooter P Gain", kP);
    SmartDashboard.putNumber("Shooter I Gain", kI);
    SmartDashboard.putNumber("Shooter D Gain", kD);
    SmartDashboard.putNumber("Shooter I Zone", kIz);
    SmartDashboard.putNumber("Shooter Feed Forward", kFF);
    SmartDashboard.putNumber("Shooter Max Output", kMinOutput);
    SmartDashboard.putNumber("Shooter Min Output", kMaxOutput);
    SmartDashboard.putNumber("PID Shooter RPM in", RPMin);
  }

  public void setPID (double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput, double RPMin) {
    this.RPMin = RPMin;
    // Set PID Values
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Note Fired!");
    initialTimestamp = Timer.getFPGATimestamp();
  }

  public boolean velocityIsInRange () {
    SmartDashboard.putNumber("shooter speed offset", Math.abs(m_shooter.getShooterEncoderVelocity() - RPMin));
    SmartDashboard.putNumber("shooter speed leniency", leniency);
    return (Math.abs(m_shooter.getShooterEncoderVelocity() - RPMin) < 50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterReference(RPMin);
    // if (velocityIsInRange()){
    SmartDashboard.putNumber("Target Shooter Speed", RPMin);
    SmartDashboard.putNumber("Actual Shooter Speed", m_shooter.getShooterEncoderVelocity());
    SmartDashboard.putBoolean("velocity in range", velocityIsInRange());
    if(velocityIsInRange()) {
      m_index.setIndexMotor(1);  
    }
  }
  
  @Override
  public void end (boolean interrupted) {
    m_shooter.setShooterSparks(0.);
    m_index.setIndexMotor(0.);
    // SmartDashboard.putNumber("Actual Shooter Speed", 0.); // It won't be actual but it'll stop unintended behavior when the command is done and deleted
    // System.out.println("Note firing completed.");
  }
}
