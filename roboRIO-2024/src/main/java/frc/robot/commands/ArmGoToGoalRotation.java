// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmFeedForwardValues;
import frc.robot.subsystems.Arm;

public class ArmGoToGoalRotation extends Command {
  PIDController rotationPIDController = new PIDController(.25, 0, 0);
  PIDController velocityPIDController = new PIDController(.25, 0, 0);
  TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ArmFeedForwardValues.maxVelocity, ArmFeedForwardValues.maxAcceleration));

  double goalRotationRad;
  Arm m_arm;
  ArmFeedforward armFeedforward;

  /** Creates a new ArmGoToGoalRotation. */
  public ArmGoToGoalRotation(Arm arm, double goalRotationRad) {
    armFeedforward = new ArmFeedforward(
      ArmFeedForwardValues.staticGain,
      ArmFeedForwardValues.gravityGain,
      ArmFeedForwardValues.velocityGain
    );
    addRequirements(arm);
    m_arm = arm;
    this.goalRotationRad = goalRotationRad;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State targetState = trapezoidProfile.calculate(
      .02, // 0.02 because each schedule takes 20ms
      new TrapezoidProfile.State(m_arm.getRotation(), m_arm.getVelocity()),
      new TrapezoidProfile.State(goalRotationRad, 0)
    );
    double voltageOutput = armFeedforward.calculate(targetState.position, targetState.velocity);
    double positionPIDOutput = rotationPIDController.calculate(m_arm.getRotation(), targetState.position);
    double velocityPIDOutput = velocityPIDController.calculate(m_arm.getVelocity(), targetState.velocity);
    
    m_arm.setSparksVoltage(MathUtil.clamp(voltageOutput + positionPIDOutput + velocityPIDOutput, -12., 12.));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setSparks(0);
  }
}
