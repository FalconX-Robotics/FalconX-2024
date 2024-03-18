// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ResetArmEncoder extends Command {
  Arm m_arm;
  /** Creates a new ResetArmEncoder. */
  public ResetArmEncoder(Arm arm) {
    addRequirements(arm);
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.resetEncoder();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
