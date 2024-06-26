// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmStayInPlace extends Command {
  Arm m_arm;
  public ArmStayInPlace(Arm arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setSparks((Math.cos(m_arm.getRotation())*Constants.ArmFeedForwardConstants.gravityGain)/12);
  }
}
