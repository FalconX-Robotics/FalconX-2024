package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.DashboardHelper;
import frc.robot.Settings;

public class Climber extends SubsystemBase{
    CANSparkMax climberSparkMax = new CANSparkMax(MotorConstants.climber, MotorType.kBrushless);
    CANSparkMax climberFollowerSparkMax = new CANSparkMax(MotorConstants.climberFollower, MotorType.kBrushless);
    Settings m_settings;

    public void setSparks (double volt) {
        climberSparkMax.set(volt);
    }

    public Climber() {
        climberSparkMax.setInverted(false);
        climberSparkMax.setSmartCurrentLimit(0);

        climberFollowerSparkMax.follow(climberSparkMax, true);
        climberFollowerSparkMax.setSmartCurrentLimit(0);
    }


    @Override
    public void periodic() {
        DashboardHelper.putNumber(LogLevel.Info, "Climber Position", climberSparkMax.getEncoder().getPosition());
    }
}
