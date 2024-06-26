package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;

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
import frc.robot.commands.ArmStayInPlace;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;

public class AimArm extends Command{
    private Optional<Double> angle;
    Arm m_arm;
    Vision m_vision;
    private double shooterHeight = 0.56; // meters
    Command moveArmCommand;

    public AimArm(Arm arm, Vision vision) {
        m_arm = arm;
        m_vision = vision;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // i cleaned up this code so that it doesn't hurt my eyes when i look at it
        // - sam madsen

        boolean isXMetersEmpty = m_vision.getXMeters().isEmpty();
        boolean isYMetersEmpty = m_vision.getYMeters().isEmpty();
        boolean isZMetersEmpty = m_vision.getZMeters().isEmpty();

        boolean condition = (!isXMetersEmpty) && (!isYMetersEmpty) && (!isZMetersEmpty);

        if (condition) {
            double metersX = m_vision.getXMeters().get();
            double metersY = m_vision.getYMeters().get();
            double distance = Math.sqrt( Math.pow(metersX, 2) + Math.pow(metersY, 2) );

            angle = Optional.of(TrajectorySim.getAngle(metersX, shooterHeight));
            double calculatedAngle = m_arm.targetAngleToArmAngle(angle.get());

            DashboardHelper.putNumber( LogLevel.Debug, "Calculated Shooting Angle", calculatedAngle );
            moveArmCommand = new ArmGoToGoalRotation( m_arm, calculatedAngle ).withTimeout(1.5) ;
            CommandScheduler.getInstance().schedule(moveArmCommand);
        } else angle = Optional.empty();
            //get angle

        // angle = m_vision.getAngleToSpeaker();
        
        
        // CommandScheduler.getInstance().schedule(/);
    }


    @Override
    public void execute() {
        boolean isXMetersEmpty = m_vision.getXMeters().isEmpty();
        boolean isYMetersEmpty = m_vision.getYMeters().isEmpty();
        boolean isZMetersEmpty = m_vision.getZMeters().isEmpty();

        boolean condition = (!isXMetersEmpty) && (!isYMetersEmpty) && (!isZMetersEmpty);
        if (condition) {
            DashboardHelper.putBoolean( LogLevel.Debug, "Has Target Angle", angle.isPresent() );
            DashboardHelper.putNumber( LogLevel.Debug, "Target Angle", angle.get() );
        }
        //Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config)
        
    }

    @Override
    public boolean isFinished() {
        if ( angle.isEmpty() ) return true;

        double arm_rotation = m_arm.getRotation();
        return !( Math.abs( arm_rotation - angle.get() ) < 1 ); // true if angle is within 1 degree
    }
    
    @Override
    public void end(boolean interrupted) {
        moveArmCommand.cancel();
        CommandScheduler.getInstance().schedule(new ArmStayInPlace(m_arm)); // for debug
        // CommandScheduler.getInstance().schedule(new ArmGoToGoalRotation(m_arm, Math.toDegrees(0.5)).withTimeout(0.8));
    }
}
