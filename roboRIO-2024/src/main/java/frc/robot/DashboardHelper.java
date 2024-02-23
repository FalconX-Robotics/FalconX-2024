package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardHelper {
    public static LogLevel curLevel;

    //1 - Important. Printed every time
    //2 - Info. Useful and Important info
    //3 - Debug. Info for Debugging
    //4 - Verbose. Everything
   public static enum LogLevel{
        Important (1),
        Info (2),
        Debug (3),
        Verbose (4);

        private int levelInt;

        private LogLevel(int level) {
            levelInt = level;
        }

        public int getLevelInt() {
            return levelInt;
        }
   }

   public static void setLogLevel(LogLevel level){
        curLevel = level;
        
   }

   public static void putNumber(LogLevel level, String key, double value){
    if(level.getLevelInt() <= curLevel.getLevelInt()){
        SmartDashboard.putNumber(key, value);
    }
   }
   public static void putBoolean(LogLevel level, String key, boolean value){
    if(level.getLevelInt() <= curLevel.getLevelInt()){
        SmartDashboard.putBoolean(key, value);
    }
   }
   public static void putString(LogLevel level, String key, String value){
    if(level.getLevelInt() <= curLevel.getLevelInt()){
        SmartDashboard.putString(key, value);
    }
   }
   public static void putData(LogLevel level, String key, Sendable value){
    if(level.getLevelInt() <= curLevel.getLevelInt()){
        SmartDashboard.putData(key, value);
    }
   }
}
