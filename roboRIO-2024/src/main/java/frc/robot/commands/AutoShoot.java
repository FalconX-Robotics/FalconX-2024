package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.TrajectorySim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AutoShoot extends Command{
    Shooter m_shooter;
    Vision m_vision;
    Index m_index;
    Command pidShootCommand;
    Arm m_arm;

    public AutoShoot(Shooter shooter, Vision camera, Index index, Arm arm) {
        m_shooter = shooter;
        m_index = index;
        m_vision = camera;
        m_arm = arm;

        addRequirements(m_shooter, m_index);
    }
    // boolean finished = false;

    @Override
    public void initialize() {
        if ((!m_vision.getXMeters().isEmpty() &&
             !m_vision.getYMeters().isEmpty() &&
             !m_vision.getZMeters().isEmpty())) {
                
                pidShootCommand = new PIDShoot(m_shooter, m_index, 75, TrajectorySim.convertMetersToRPM( TrajectorySim.getVelocity(Math.sqrt(
                Math.pow(m_vision.getXMeters().get(), 2)
                + Math.pow(m_vision.getZMeters().get(), 2)),
            0.5))).withTimeout(2.5);
            CommandScheduler.getInstance().schedule(pidShootCommand);
        }
    }
/*
 * 
 * TrajectorySim.convertMetersToRPM( TrajectorySim.getVelocity(Math.sqrt(
                Math.pow(m_vision.getXMeters().get(), 2)
                + Math.pow(m_vision.getZMeters().get(), 2)),
            TrajectorySim.speakerHeight)))
 */
    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
    @Override
    public void end (boolean interrupted) {
        if (pidShootCommand != null) {
            pidShootCommand.cancel();
        }
        m_shooter.setShooterSparks(0.);
        m_index.setIndexMotor(0.);
    }
}
