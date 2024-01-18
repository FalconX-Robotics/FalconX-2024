package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Settings {
    private static XboxController m_driveController;
    private static XboxController m_noteController;
    Settings (XboxController driveController, XboxController noteController){
        m_driveController = driveController;
        m_noteController = noteController;
    }
    public static class DriveController {
        public static double driveControllerDeadband = 0.1;
        public static boolean getTurnInPlaceButton () {return m_driveController.getAButtonPressed();}
        public static double getSpeedJoystick () {return m_driveController.getLeftY();}
        public static double getRotationJoystick () {return m_driveController.getRightX();}
    }
    public static class NoteController {
        public static boolean getShooterButton () {return m_noteController.getAButtonPressed();}
        public static double getArmJoystick () {return m_noteController.getRightX();}
    }
}
