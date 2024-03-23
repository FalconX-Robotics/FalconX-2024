package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.DashboardHelper;
import frc.robot.Constants.DIOConstants;

public class LimitSwitch extends SubsystemBase { 
    DigitalInput input;

    public LimitSwitch (int id) {
        input = new DigitalInput(id);
    }
    // DataLog log = DataLogManager.getLog();
    // BooleanLogEntry sensorOnLog = new BooleanLogEntry(log, "/limit_switches/limit_switches");
    
    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("limit switch on", !input.get());
        // DashboardHelper.putBoolean(DashboardHelper.LogLevel.Info, "sensor on", !input.get());
        // System.out.println(input.get());
    }
    public boolean getAsBoolean() {
        // sensorOnLog.append(!input.get());
        return input.get();
    }
}
