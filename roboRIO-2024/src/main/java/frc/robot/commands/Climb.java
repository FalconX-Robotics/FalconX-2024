package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
    Climber m_climber;
    double volt;

    public Climb(Climber climber, double volt) {
        m_climber = climber;
        this.volt = volt;
    }

    @Override
    public void execute() {
        m_climber.setSparks(volt);
    }
}
