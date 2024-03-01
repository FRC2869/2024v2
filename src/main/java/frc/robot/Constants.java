package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Constants {
    public static Timer timer = new Timer();
    public static class OperatorConstants {
        public static interface DriverController{
            public static final int port = 0;
            public static final double leftXDeadband = .01;
            public static final double leftYDeadband = .01;
            public static final double rightXDeadband = .01;
            public static final double righYDeadband = .01;
        }        
        public static interface OperatorController{
            public static final int port = 2;
            public static final double leftXDeadband = .01;
            public static final double leftYDeadband = .01;
            public static final double rightXDeadband = .01;
            public static final double righYDeadband = .01;
        } 
        public static interface OperatorButtonBoard{
            public static final int port = 3;
        } 
    }
    public static class ShooterConstants {

        public static final int id2 = 18;
        public static final int id1 = 19;
        public static final MotorConfiguration config = new MotorConfiguration(1.0, 0, 10, 0, 0, 0.5, -0.5, 0, 0, 40, true, false, 0);

    }
}
