// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SimpleShoot extends Command {
  Shooter m_shooter;
  /** Creates a new SimpleShoot. */
  public SimpleShoot(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterSparks(.25);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterSparks(0.);
  }
}
