package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Sensors extends SubsystemBase { 
    DigitalInput input = new DigitalInput(0);

    DataLog log = DataLogManager.getLog();
    BooleanLogEntry sensorOnLog = new BooleanLogEntry(log, "/sensors/sensor_on");
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("sensor on", input.get());
        sensorOnLog.append(input.get());
        System.out.println(input.get());
    }
}
