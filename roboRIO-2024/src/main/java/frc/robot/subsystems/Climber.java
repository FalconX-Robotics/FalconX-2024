package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.commands.ClimbIndividual.Side;
import frc.robot.DashboardHelper;
import frc.robot.Settings;

public class Climber extends SubsystemBase{
    CANSparkMax leftClimberSparkMax = new CANSparkMax(MotorConstants.leftClimber, MotorType.kBrushless);
    CANSparkMax rightClimberSparkMax = new CANSparkMax(MotorConstants.rightClimber, MotorType.kBrushless);
    Settings m_settings;

    public void setSparks (double volt) {
        leftClimberSparkMax.set(volt);
        rightClimberSparkMax.set(volt);
    }
    public void setOneSide (Side side, double volt) {
        switch (side) {
            case LEFT:
            leftClimberSparkMax.set(volt);
            break;
            case RIGHT:
            rightClimberSparkMax.set(volt);
            break;
        }
    }

    LimitSwitch m_leftLimitSwitch, m_rightLimitSwitch;
    public Climber(LimitSwitch leftLimitSwitch, LimitSwitch rightLimitSwitch) {
        m_leftLimitSwitch = leftLimitSwitch;
        m_rightLimitSwitch = rightLimitSwitch;

        leftClimberSparkMax.setIdleMode(IdleMode.kBrake);
        leftClimberSparkMax.setInverted(false);
        leftClimberSparkMax.setSmartCurrentLimit(50);
        leftClimberSparkMax.burnFlash();

        rightClimberSparkMax.setIdleMode(IdleMode.kBrake);
        rightClimberSparkMax.setInverted(false);
        // rightClimberSparkMax.follow(climberSparkMax, true);
        rightClimberSparkMax.setSmartCurrentLimit(50);
        rightClimberSparkMax.burnFlash();
    }

    @Override
    public void periodic() {
        DashboardHelper.putNumber(LogLevel.Info, "Left Climber Position", leftClimberSparkMax.getEncoder().getPosition());
        DashboardHelper.putNumber(LogLevel.Info, "Right Climber Position", rightClimberSparkMax.getEncoder().getPosition());
    }
}
