package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * <a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf"> LED Colors
 */
public class LEDs extends SubsystemBase{

    private CANSparkMax LEDs;

    public LEDs() {
        LEDs = new CANSparkMax(Constants.LED_PORT, MotorType.kBrushless);
    }

    @Override
    public void register() {
        LEDs.set(0.91);
        super.register();
    }
}
