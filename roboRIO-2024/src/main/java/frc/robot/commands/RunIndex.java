// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class RunIndex extends Command {
  Index m_index;
  double velocity;
  /** Creates a new RunIndex. */
  public RunIndex(Index index, double velocity) {
    m_index = index;
    this.velocity = velocity;
    addRequirements(m_index);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_index.setIndexMotor(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.setIndexMotor(0.);
  }
}
