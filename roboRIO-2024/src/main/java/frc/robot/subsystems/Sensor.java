package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sensor extends SubsystemBase { 
    DigitalInput input = new DigitalInput(Constants.SENSOR_PORT);
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("sensor on", input.get());
        // System.out.println(input.get());
    }
    public boolean getNoteSensed() {
        return input.get();
    }
}
