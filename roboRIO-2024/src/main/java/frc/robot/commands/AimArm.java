package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DashboardHelper;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;

public class AimArm extends Command{
    private Optional<Double> angle;
    Arm m_arm;
    Vision m_vision;

    public AimArm(Arm arm, Vision vision) {
        m_arm = arm;
        m_vision = vision;
        addRequirements(arm, vision);
    }

    @Override
    public void initialize() {
        angle = m_vision.getAngleToSpeaker();
        DashboardHelper.putNumber(LogLevel.Debug, "Target Angle", angle.isPresent()?angle.get():0);
        new ArmGoToGoalRotation(m_arm, Math.toRadians(angle.get()));
        // CommandScheduler.getInstance().schedule(/);
    }

    @Override
    public void execute() {
        DashboardHelper.putNumber(LogLevel.Debug, "Target Angle", angle.isPresent()?angle.get():0);
    }

    @Override
    public boolean isFinished() {
        return angle.isPresent()
        ? (Math.abs(m_arm.getRotation() - Math.toRadians(angle.get())) < 1)
        : false;
    }
}
