// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import frc.robot.DashboardHelper;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardPIDShoot extends Command {
  private Shooter m_shooter;
  private SparkPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput, maxRPM;

  /** Creates a new Shoot. */
  public SmartDashboardPIDShoot(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
    m_pidController = m_shooter.getShooterPidController();
    setPID(6e-5, 0, 0, 0, 0.000173, -1, 1);

    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Shooter I Gain", 0);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Shooter D Gain", 0);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Shooter I Zone", 0);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Shooter Feed Forward", 0);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Shooter Max Output", 0);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Shooter Min Output", 0);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Shooter P Gain", 0); 
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
    System.out.println("yeet");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setPID(
      DashboardHelper.getNumber(DashboardHelper.LogLevel.Important, "Shooter P Gain", 0),
      DashboardHelper.getNumber(DashboardHelper.LogLevel.Important, "Shooter I Gain", 0), 
      DashboardHelper.getNumber(DashboardHelper.LogLevel.Important, "Shooter D Gain", 0), 
      DashboardHelper.getNumber(DashboardHelper.LogLevel.Important, "Shooter I Zone", 0), 
      DashboardHelper.getNumber(DashboardHelper.LogLevel.Important, "Shooter Feed Forward", 0), 
      DashboardHelper.getNumber(DashboardHelper.LogLevel.Important, "Shooter Min Output", 0), 
      DashboardHelper.getNumber(DashboardHelper.LogLevel.Important, "Shooter Max Output", 0)
    );
    m_shooter.setShooterReference(2700);

  }
}
