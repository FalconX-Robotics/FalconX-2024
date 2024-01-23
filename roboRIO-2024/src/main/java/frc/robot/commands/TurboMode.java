package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TurboMode extends Command{
     Drivetrain m_drivetrain;

    public TurboMode(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        m_drivetrain.turboModeOn = true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.turboModeOn = false;
    }
}
