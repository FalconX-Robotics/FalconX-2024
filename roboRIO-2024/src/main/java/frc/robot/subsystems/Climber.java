package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings;

public class Climber extends SubsystemBase{
    CANSparkMax climberSparkMax = new CANSparkMax(MotorConstants.climber, MotorType.kBrushless);
    Settings m_settings;

    public void setClimberSparkMax(double volt) {
        climberSparkMax.set(volt);
    }

    public Climber() {
        climberSparkMax.setInverted(false);
        climberSparkMax.setSmartCurrentLimit(0);

        climberSparkMax.setInverted(true);
        climberSparkMax.setSmartCurrentLimit(0);

    }


    @Override
    public void periodic() {
        
    }
}
