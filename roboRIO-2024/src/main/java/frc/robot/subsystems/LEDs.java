package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * <a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf"> LED Colors
 */
public class LEDs extends SubsystemBase{


    private final SendableChooser<LEDs.Color> colorChooser = new SendableChooser<>();

    private Spark LEDs = new Spark(Constants.LED_PORT);
    private Color color = Color.PURPLE;

    public LEDs() {
        //?
        // LEDs = new Spark(Constants.LED_PORT);
    }

    @Override
    public void periodic() {
        LEDs.set(color.getValue());
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public static enum Color {
        PURPLE (0.91),
        HEARTBEAT_RED (-0.25),
        HEARTBEAT_BLUE (-0.23),
        YELLOW (0.69);

        private final double value;

        Color(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }
}
