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

    public static class PivotConstants {
        public static final int id = 17;
        public static final MotorConfiguration config = new MotorConfiguration(1.0, 0, 0, 0, 0, .3, -.3, ((30/18.0)*(56.0/30.0)*(25)), 0, 40, false, false, 0);
        public static final int basePosition = 0;
        public static final double kMinAngle = 0;
        public static final double kMaxAngle = 0;
        public static final double ampPosition = 100;
        public static final double farPosition = 0;
        public static enum PositionsPivot{
            BASE, AMP, FAR,
        }
    }
}
