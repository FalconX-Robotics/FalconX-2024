// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class PIDShoot extends Command {
  private Index m_index;
  private Shooter m_shooter;
  private SparkPIDController m_pidController;
  private double RPMin, leniency, initialTimestamp;

  /** Creates a new Shoot. */
  public PIDShoot(Index index, Shooter shooter) {
    this(index, shooter, 35., 2700.);
  }

  public PIDShoot(Index index, Shooter shooter, double leniency, double RPMin) {
    this(index, shooter, 6e-5, 0, 0, 0, 0.000173, -1, 1, RPMin);
  }

  public PIDShoot (Index index, Shooter shooter, double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput, double RPMin) {
    m_index = index;
    m_shooter = shooter;
    m_pidController = m_shooter.getShooterPidController();
    addRequirements(index, shooter);
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
    System.out.println("Note Fired!");
    initialTimestamp = Timer.getFPGATimestamp();
  }

  public boolean velocityIsInRange () {
    return (m_shooter.getShooterEncoderVelocity() >= RPMin - leniency
         && m_shooter.getShooterEncoderVelocity() <= RPMin + leniency);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterReference(RPMin);
    // if (velocityIsInRange()){
      if (Timer.getFPGATimestamp() >= 1 + initialTimestamp){
        new RunIndex(m_index, 1.).until(() -> {return Timer.getFPGATimestamp() >= initialTimestamp + 3;});
      }
      // }
    SmartDashboard.putNumber("Timestamp", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Timestamp inital", initialTimestamp);
    
  }
  
  @Override
  public void end (boolean interrupted) {
    m_shooter.setShooterSparks(0.);
    System.out.println("Note firing completed.");
  }
}
