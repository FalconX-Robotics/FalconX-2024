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
        DashboardHelper.putNumber(LogLevel.Debug, "Left Climb set", volt);
        DashboardHelper.putNumber(LogLevel.Debug, "Right Climb set", volt);
        leftClimberSparkMax.set(volt);
        rightClimberSparkMax.set(volt);
    }
    public void setOneSide (Side side, double volt) {
        switch (side) {
            case LEFT:
            DashboardHelper.putNumber(LogLevel.Debug, "Left Climb set", volt);
            leftClimberSparkMax.set(volt);
            break;
            case RIGHT:
            DashboardHelper.putNumber(LogLevel.Debug, "Right Climb set", volt);
            rightClimberSparkMax.set(volt);
            break;
        }
    }

    LimitSwitch m_leftLimitSwitch, m_rightLimitSwitch;

    public boolean climberIsDown (Side side) {
        switch (side) {
            case LEFT:
            return !m_leftLimitSwitch.getAsBoolean();
            case RIGHT:
            return !m_rightLimitSwitch.getAsBoolean();
            default:
            return false;
        }
    }

    public Climber(LimitSwitch leftLimitSwitch, LimitSwitch rightLimitSwitch) {
        m_leftLimitSwitch = leftLimitSwitch;
        m_rightLimitSwitch = rightLimitSwitch;

        leftClimberSparkMax.setIdleMode(IdleMode.kBrake);
        leftClimberSparkMax.setInverted(true);
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
        DashboardHelper.putBoolean(LogLevel.Info, "Left Climber Is Down", climberIsDown(Side.LEFT));
        DashboardHelper.putBoolean(LogLevel.Info, "Right Climber Is Down", climberIsDown(Side.RIGHT));
    }
}
