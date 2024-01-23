// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends Command {
  Settings m_settings;
  Drivetrain m_drivetrain;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain drivetrain, Settings settings) {
    m_settings = settings;
    m_drivetrain = drivetrain;
  addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_settings.driveController.getSpeedJoystickValue(), m_settings.driveController.getRotationJoystickValue());
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
