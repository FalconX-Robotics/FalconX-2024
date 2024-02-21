// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Index extends SubsystemBase {
  CANSparkMax indexSparkMax = new CANSparkMax(MotorConstants.index, MotorType.kBrushless);
  /** Creates a new Index. */
  public Index() {
    indexSparkMax.setInverted(false); // change later if necessary
  }

  public void setIndexMotor(double velocity) {
    indexSparkMax.set(velocity);
    System.out.println("setting index to " + velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
