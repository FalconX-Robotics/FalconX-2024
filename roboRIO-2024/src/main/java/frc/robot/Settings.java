package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 
public class Settings {
    // Access Controllers
    private XboxController m_driveController, m_noteController;
    Settings (XboxController driveController, XboxController noteController){
        m_driveController = driveController;
        m_noteController = noteController;

    }

    // Do not remove. or else
    public DriveController driveController = new DriveController();
    public NoteController noteController = new NoteController();

    public static double curveInput(double input) {
        // double curveAmount = SmartDashboard.getNumber("Input Curve Amount", Constants.OperatorConstants.INPUT_CURVE_AMOUNT);
        return Math.abs(Math.pow(input, Constants.OperatorConstants.INPUT_CURVE_AMOUNT)) * Math.signum(input);
    }

    public FeedForwardValues feedForwardValues = new FeedForwardValues();

    /** Configurations for controller centered around drivetrain repositioning */
    public class DriveController {
        public double getSpeedJoystickValue () { 
            double output = -m_driveController.getLeftY();
            output = MathUtil.applyDeadband(output, deadband);
            output = curveInput(output);
            return output;
        }
        public double getRotationJoystickValue () {
 return MathUtil.applyDeadband( 
            m_driveController.getRightX(), deadband);
        }
        public boolean getTurboButton () {return m_driveController.getRightBumper();}
        public boolean getTurnInPlaceButton () {return m_driveController.getLeftBumper();}

        public double deadband = 0.1;
        public double normalSpeed = 0.3;
        public double turboSpeed = 1.;
    }
    /** Configurations for controller centered around note manipulation */
    public class NoteController {
        public Button getShooterButton () {return XboxController.Button.kA;}
        public Button getIntakeButton  () {return XboxController.Button.kB;}
        public Button getIndexButton   () {return XboxController.Button.kB;}
        public Button getReverseButton () {return XboxController.Button.kRightBumper;}

        public double getArmJoystickValue () {return MathUtil.applyDeadband( 
            m_noteController.getLeftY(), deadband);
        }
        public double deadband = 0.1;
    }
    /** To be changed later; change values based on the weight, offset, etc of the arm */
    public class FeedForwardValues {
        /** In radians, the offset of the arm */
        public final double offset = 10.;
        public final double staticGain = 1.;
        public final double gravityGain = 1.;
        public final double velocityGain = 1.;
    }
}
