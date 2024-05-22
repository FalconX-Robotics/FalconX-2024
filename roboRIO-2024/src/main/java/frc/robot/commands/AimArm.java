package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DashboardHelper;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.simulation.TrajectorySim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;

public class AimArm extends Command{
    private Optional<Double> angle;
    Arm m_arm;
    Vision m_vision;
    private double shotHeight = 0.5; // meters

    public AimArm(Arm arm, Vision vision) {
        m_arm = arm;
        m_vision = vision;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (!m_vision.getXMeters().isEmpty() &&
            !m_vision.getYMeters().isEmpty() &&
            !m_vision.getZMeters().isEmpty()) {
                angle = Optional.of(TrajectorySim.getAngle(
                    Math.sqrt(
                        Math.pow(m_vision.getXMeters().get(), 2)
                        + Math.pow(m_vision.getZMeters().get(), 2)),
                    0.5)
                );
            } else {angle = Optional.empty();}
            //get angle

        // angle = m_vision.getAngleToSpeaker();
        DashboardHelper.putNumber(LogLevel.Debug, "shooting angle after calcs", m_arm.targetAngleToArmAngle(angle.get()));
        CommandScheduler.getInstance().schedule(new ArmGoToGoalRotation(m_arm, m_arm.targetAngleToArmAngle(angle.get())));
        
        // CommandScheduler.getInstance().schedule(/);
    }


    @Override
    public void execute() {
        DashboardHelper.putBoolean(LogLevel.Debug, "has target angle", angle.isPresent());
        DashboardHelper.putNumber(LogLevel.Debug, "target angle", angle.get());
        //Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config)
        
    }

    @Override
    public boolean isFinished() {
        if (angle.isEmpty()) return true;
        return !(Math.abs(m_arm.getRotation() - angle.get()) < 1); //true if angle is within 1 degree
    }
}
