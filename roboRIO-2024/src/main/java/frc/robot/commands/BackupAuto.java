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
        
        double time = Timer.getFPGATimestamp();

        if(time < 5) {
            m_drivetrain.setMotors(0.2, 0.2);
        } else {
            m_drivetrain.setMotors(0.0, 0.0);
        }
    }
    @Override
    public void execute() {
       moveFoward();
    }
}
