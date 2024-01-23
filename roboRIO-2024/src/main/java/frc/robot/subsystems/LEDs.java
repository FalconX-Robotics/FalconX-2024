package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * <a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf"> LED Colors
 */
public class LEDs extends SubsystemBase{


    private final SendableChooser<LEDs.Color> colorChooser = new SendableChooser<>();

    private CANSparkMax LEDs;
    private Color color = Color.PURPLE;

    public LEDs() {
        LEDs = new CANSparkMax(Constants.LED_PORT, MotorType.kBrushless);

        colorChooser.addOption("solid purple", Color.PURPLE);
        colorChooser.addOption("flashing red", Color.HEARTBEAT_RED);
        colorChooser.addOption("flashing blue", Color.HEARTBEAT_BLUE);
    }

    @Override
    public void periodic() {
        LEDs.set(colorChooser.getSelected().getValue());
    }

    public static enum Color {
        PURPLE (0.91),
        HEARTBEAT_RED (-0.25),
        HEARTBEAT_BLUE (-0.23);

        private final double value;

        Color(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }
}
