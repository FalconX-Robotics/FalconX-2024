// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax shooterArmSparkMax = new CANSparkMax(MotorConstants.shooterArm, MotorType.kBrushless);
  CANSparkMax shooterFollowerSparkMax = new CANSparkMax(MotorConstants.shooterFollower, MotorType.kBrushless);
  CANSparkMax shooterSparkMax = new CANSparkMax(MotorConstants.shooter, MotorType.kBrushless);
  Settings m_settings;

  public void setArmSpark(double volt){
    shooterArmSparkMax.set(volt);
  }

  public void setShooterSpark(double volt){
    shooterSparkMax.set(volt);
  }

  public SparkPIDController getShooterPidController () {
    return shooterSparkMax.getPIDController();
  }

  public double getShooterArmEncoderRotation() {
    return shooterArmSparkMax.getEncoder().getPosition();
  }

  public void setFeedForward (double idk){
    // new ArmFeedforward(idk, idk, idk);
    //TODO: figure it out.
    // shooterArmSparkMax.set();
  }

  /** Creates a new Shooter. */
  public Shooter(Settings settings) {
    m_settings = settings;
    shooterArmSparkMax.getEncoder().setPositionConversionFactor(1.);
    shooterArmSparkMax.getEncoder().setPosition(0);
    shooterArmSparkMax.setInverted(false);
    // TODO: Change position conversion factor as needed
    shooterArmSparkMax.getEncoder().setPosition(0); // resets position of encoder
    shooterArmSparkMax.set(
      limitArmViaEncoder(
        m_settings.noteController.getArmJoystickValue()
      )
    );
  }

  /**
   * A method to block the arm from moving too far.
   * @param input A double from -1 to 1. Put joystick value here
   * @return The same double, unless moved too far.
   */
  private double limitArmViaEncoder (double input){
    // TODO: set the position temporary values under this line.
    if (shooterArmSparkMax.getEncoder().getPosition() >= 1000. && input >= 0.) {
      return 0.;
    }
    if (shooterArmSparkMax.getEncoder().getPosition() <= -1000. && input <= 0.) {
      return 0.;
    }
    return input;
  }

  public void setShooterReference(SparkPIDController pidController, double setPoint) {
    pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setMotors(double speed){
    shooterSparkMax.set(speed);
    shooterFollowerSparkMax.set(speed);
  }

  @Override
  public void periodic() {
    // If no current command, set arm via joystick value.
    if(this.getCurrentCommand() == null) {
      shooterArmSparkMax.set(
        limitArmViaEncoder(
          m_settings.noteController.getArmJoystickValue()
        )
      );
    }
    // This method will be called once per scheduler run
  }
}