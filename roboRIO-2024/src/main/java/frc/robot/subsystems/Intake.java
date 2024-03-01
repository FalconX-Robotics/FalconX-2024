// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeSparkMax = new CANSparkMax(MotorConstants.intake, MotorType.kBrushless);
  
  /** Creates a new Intake. */
  public Intake() {
    intakeSparkMax.restoreFactoryDefaults();
    intakeSparkMax.setSmartCurrentLimit(40);
    intakeSparkMax.setInverted(true);
    intakeSparkMax.burnFlash();
  }

  public void setMotor(double speed){
    System.out.println("setting intake to "+speed);
    intakeSparkMax.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
