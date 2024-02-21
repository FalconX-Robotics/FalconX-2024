// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PIDShoot extends Command {
  private Shooter m_shooter;
  private SparkPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput, maxRPM;

  /** Creates a new Shoot. */
  public PIDShoot(Shooter shooter) {
    this(shooter, 6e-5, 0, 0, 0, 0.000173, -1, 1);
  }

  public PIDShoot (Shooter shooter, double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput) {
    m_shooter = shooter;
    m_pidController = m_shooter.getShooterPidController();
    addRequirements(shooter);
    setPID(kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
    SmartDashboard.putNumber("Shooter P Gain", kP);
    SmartDashboard.putNumber("Shooter I Gain", kI);
    SmartDashboard.putNumber("Shooter D Gain", kD);
    SmartDashboard.putNumber("Shooter I Zone", kIz);
    SmartDashboard.putNumber("Shooter Feed Forward", kFF);
    SmartDashboard.putNumber("Shooter Max Output", kMinOutput);
    SmartDashboard.putNumber("Shooter Min Output", kMaxOutput);
    SmartDashboard.putNumber("PID Shooter RPM in", 2700.);
  }

  public void setPID (double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput) {
    this.kP = kP; this.kI = kI; this.kD = kD;
    this.kIz = kIz; this.kFF = kFF;
    this.kMinOutput = kMinOutput; this.kMaxOutput = kMaxOutput;
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
    System.out.println("Note Fired!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterReference(SmartDashboard.getNumber("PID Shooter RPM in", 2700.));
  }
  @Override
  public void end (boolean interrupted) {
    m_shooter.setMotors(0.);
    System.out.println("Note firing completed.");
  }
}
