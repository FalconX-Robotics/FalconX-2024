package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbUp extends Command {
    Climber m_climber;
    double volt;

    public ClimbUp(Climber climber, double volt) {
        this.volt = volt;
    }

    @Override
    public void execute() {
        m_climber.setClimberSparkMax(volt);
    }
}
