package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;

public class AimArm extends Command{
    private Optional<Double> angle;
    Arm m_arm;
    Vision m_vision;
    boolean finished = false;

    public AimArm(Arm arm, Vision vision) {
        m_arm = arm;
        m_vision = vision;
        addRequirements(arm, vision);
    }

    @Override
    public void initialize() {
        angle = m_vision.getAngleToSpeaker();
        new ArmGoToGoalRotation(m_arm, angle.get());
        // CommandScheduler.getInstance().schedule(/);
    }

    @Override
    public void execute() {
        if(angle.isPresent()) {
            finished = !(Math.abs(m_arm.getRotation() - angle.get()) < 1);
        } else {
            finished = true;
        }
        
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
