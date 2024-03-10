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

  /** @return The trigger value of whichever joystick is pushed in more */
  private double getGreatestTriggerValue(){
    final double firstTrigger = m_settings.noteSettings.getUpClimbValue();
    final double secondTrigger = m_settings.noteSettings.getDownClimbValue();
    return firstTrigger>secondTrigger?firstTrigger:secondTrigger;
  }

  @Override
  public void execute() {
    m_climber.setSparks(getGreatestTriggerValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setSparks(0.);
  }
}
