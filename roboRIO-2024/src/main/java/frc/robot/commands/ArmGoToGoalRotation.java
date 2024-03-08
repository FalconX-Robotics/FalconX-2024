// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DashboardHelper;
import frc.robot.Constants.ArmFeedForwardConstants;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.subsystems.Arm;

public class ArmGoToGoalRotation extends Command {
  PIDController rotationPIDController = new PIDController(30., 0, 0);
  // PIDController velocityPIDController = new PIDController(0., 0, 0);
  TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(ArmFeedForwardConstants.maxVelocity, ArmFeedForwardConstants.maxAcceleration));
  TrapezoidProfile.State currentState = new TrapezoidProfile.State(0., 0.);
  TrapezoidProfile.State targetState = new TrapezoidProfile.State(0., 0.);
  double goalRotationRad;
  Arm m_arm;
  ArmFeedforward armFeedforward;

  /** Creates a new ArmGoToGoalRotation. */
  public ArmGoToGoalRotation(Arm arm, double goalRotationRad) {
    armFeedforward = new ArmFeedforward(
      ArmFeedForwardConstants.staticGain,
      ArmFeedForwardConstants.gravityGain,
      ArmFeedForwardConstants.velocityGain
    );
    addRequirements(arm);
    m_arm = arm;
    this.goalRotationRad = goalRotationRad;
  }

  @Override
  public void initialize () {
    targetState = new TrapezoidProfile.State(m_arm.getRotation(), m_arm.getVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetState = trapezoidProfile.calculate(
      .02, // 0.02 because each schedule takes 20ms
      targetState,
      new TrapezoidProfile.State(goalRotationRad, 0)
    );
    double voltageOutput = armFeedforward.calculate(targetState.position - ArmFeedForwardConstants.offset, targetState.velocity);
    double positionPIDOutput = rotationPIDController.calculate(m_arm.getRotation(), targetState.position);
    // double velocityPIDOutput = velocityPIDController.calculate(m_arm.getVelocity(), targetState.velocity);
    DashboardHelper.putNumber(LogLevel.Important, "Target State Position", targetState.position);
    DashboardHelper.putNumber(LogLevel.Important, "Target State Velocity", targetState.velocity);

    // Why do we need to clamp? We already have a max voltage -w
    m_arm.setSparksVoltage(MathUtil.clamp(voltageOutput + positionPIDOutput, -4., 4.));
    DashboardHelper.putNumber(LogLevel.Debug, "Attempt Spark Volt", voltageOutput + positionPIDOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setSparksVoltage(0);
  }
}
