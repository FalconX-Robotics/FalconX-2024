package frc.robot.commands;

// TODO: Remove this?? What even is this??
import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AmpShoot extends Command {
    double startTime = Timer.getFPGATimestamp();
    Shooter m_shooter;

    public AmpShoot(Shooter shooter) {
        this.m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        m_shooter.setShooterSpark(0.1);
    }

    @Override
    public boolean isFinished() {
        //ends the command after half a second
        return Timer.getFPGATimestamp() - startTime >= 0.5;
    }
}
