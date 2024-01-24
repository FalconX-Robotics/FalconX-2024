package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase { 
    DigitalInput input = new DigitalInput(0);
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("sensor on", input.get());
        System.out.println(input.get());
    }
}
