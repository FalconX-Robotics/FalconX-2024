package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbIndividual extends Command {
    Climber m_climber;
    double volt;

    public enum Side {
        LEFT, RIGHT
    }
    Side side;
    /** Command for setting a specific side of the climber */
    public ClimbIndividual(Climber climber, double volt, Side side) {
        m_climber = climber;
        this.volt = volt;
        this.side = side;
    }

    @Override
    public void execute() {
        m_climber.setOneSide(side, volt);
    }
}
