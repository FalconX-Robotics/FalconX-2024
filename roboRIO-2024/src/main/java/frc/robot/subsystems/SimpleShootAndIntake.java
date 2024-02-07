// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;

public class SimpleShootAndIntake extends Command {
  Shooter m_shooter;
  Settings m_settings;
  /** Creates a new SimpleShoot. */
  public SimpleShootAndIntake(Shooter shooter, Settings settings) {
    m_settings = settings;
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_settings.noteController.getShooterButtonValue()){
      m_shooter.shooterSparkMax.set(1.);
    }
    if(m_settings.noteController.getIntakeButtonValue()){
      m_shooter.shooterSparkMax.set(1.);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
