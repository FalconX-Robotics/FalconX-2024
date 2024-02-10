// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
  CANSparkMax topIntakeSparkMax = new CANSparkMax(MotorConstants.topIntake, MotorType.kBrushless);
  CANSparkMax bottomIntakeSparkMax = new CANSparkMax(MotorConstants.bottomIntake, MotorType.kBrushless);
  
  public void setSparks(double volt) {
    topIntakeSparkMax.set(volt);
  }
  
  /** Creates a new Intake. */
  public Intake() {
    bottomIntakeSparkMax.follow(topIntakeSparkMax);
    // TODO: Set inverted here if necessary
  }

  public void setMotors(double speed){
    topIntakeSparkMax.set(speed);
    bottomIntakeSparkMax.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
