package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;

public class ChooseLEDColor extends Command {
    LEDs.Color color;
    LEDs leds;

    public ChooseLEDColor(LEDs.Color color, LEDs leds) {
        addRequirements(leds);
        
        this.leds = leds;
        this.color = color;
    }

    @Override
    public void initialize() {
        leds.setColor(color);
    }
}
