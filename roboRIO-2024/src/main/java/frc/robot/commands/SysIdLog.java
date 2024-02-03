package frc.robot.commands;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class SysIdLog extends Command{

    Drivetrain drivetrain;

    SysIdLog(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // drivetrain.logMotors();
    }
}
