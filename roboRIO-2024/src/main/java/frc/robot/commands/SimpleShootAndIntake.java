// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SimpleShootAndIntake extends Command {
  Intake m_intake;
  Shooter m_shooter;
  Settings m_settings;
  int temporary = 0;

  /** Creates a new SimpleShoot. */
  public SimpleShootAndIntake(Shooter shooter, Intake intake, Settings settings) {
    m_settings = settings;
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_settings.noteController.getShooterButtonValue()){
      // m_shooter.setMotors(1.);
    // }
    // if(m_settings.noteController.getIntakeButtonValue()){
      m_intake.setMotors(.5);
    // }
    // SmartDashboard.putBoolean("intakeButton", m_settings.noteController.getIntakeButtonValue());
    // temporary++;
    // SmartDashboard.putBoolean("tempChanged", SmartDashboard.getNumber("tempNum", temporary) != temporary);
    // SmartDashboard.putNumber("tempNum", temporary);
    // SmartDashboard.putBoolean("other button test", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotors(0.);
    m_shooter.setMotors(0.);
    System.out.println("ended motors");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
