package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class BackupAuto extends Command {
    Drivetrain m_drivetrain;
    public BackupAuto(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);

    }
    public void moveFoward() {
        m_drivetrain.setMotors(0.2, 0.2);
    }
    @Override
    public void execute() {
       moveFoward();
    }
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= 5.;
    }
    @Override   
    public void end (boolean interrupted){
        m_drivetrain.setMotors(0.,0.);
    }
}
