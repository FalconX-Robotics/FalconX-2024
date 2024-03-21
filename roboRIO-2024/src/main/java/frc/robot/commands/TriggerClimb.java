// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Climber;

public class TriggerClimb extends Command {
  Settings m_settings;
  Climber m_climber;

  /** Creates a new TriggerClimb. */
  public TriggerClimb(Settings settings, Climber climber) {
    m_settings = settings;
    m_climber = climber;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    m_climber.setSparks(m_settings.noteSettings.getGreatestTriggerValue() * .3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setSparks(0.);
  }
}
