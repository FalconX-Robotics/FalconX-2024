// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.commands.ClimbIndividual.Side;
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
    if(m_settings.noteSettings.moveClimbUpTrigger.getAsBoolean()){
      m_climber.setSparks(m_settings.noteSettings.moveClimberUpPercent);
      return;
    }
    if(!m_climber.climberIsDown(Side.LEFT)){
      m_climber.setOneSide(Side.LEFT, m_settings.noteSettings.getLeftClimbValue());
    } else {
      m_climber.setOneSide(Side.LEFT, 0.);
    }
    if(!m_climber.climberIsDown(Side.RIGHT)){
      m_climber.setOneSide(Side.RIGHT, m_settings.noteSettings.getRightClimbValue());
    } else {
      m_climber.setOneSide(Side.RIGHT, 0.);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setSparks(0.);
  }
}
