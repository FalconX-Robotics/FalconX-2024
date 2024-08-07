package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.DashboardHelper;
import frc.robot.Constants.DIOConstants;

public class Sensor extends SubsystemBase { 
    DigitalInput input = new DigitalInput(DIOConstants.SENSOR_PORT);

    DataLog log = DataLogManager.getLog();
    BooleanLogEntry sensorOnLog = new BooleanLogEntry(log, "/sensors/sensor_on");
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("sensor on", !input.get());
        DashboardHelper.putBoolean(DashboardHelper.LogLevel.Info, "sensor on", !input.get());
        // System.out.println(input.get());
    }
    public boolean getNoteSensed() {
        sensorOnLog.append(!input.get());
        return !input.get();
    }
}
