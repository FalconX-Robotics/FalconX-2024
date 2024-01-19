// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings.NoteController;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  Intake m_intake;
  XboxController m_noteController;
  double volt;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, XboxController noteController, double volt) {
    m_intake = intake;
    m_noteController = noteController;
    this.volt = volt;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setSparks(volt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSparks(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return NoteController.getShooterButton();
  }
}
