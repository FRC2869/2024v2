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

    public static class IntakeConstants {
        public static final MotorConfiguration spinMotorConfig = new MotorConfiguration(.5, -.5, 20, true, false);
        public static final MotorConfiguration pivotMotorConfig = new MotorConfiguration(1.0, 0, 0, 0, 0, 1, -1, 5*(50/16.0)*(32/16.0), 0, 40, false, false, 0);
        public static final double kMinAngle = 0;
        public static final double kMaxAngle = 0;
        public static final double floorPosition = 0;
        public static final double basePosition = 0;
        public static enum PositionsIntake {
            BASE, FLOOR
        }
        public static double getTargetPos(PositionsIntake intakeEnum) {
            switch(intakeEnum) {
                case BASE:
                    return IntakeConstants.basePosition;
                case FLOOR:
                    return IntakeConstants.floorPosition;
                default:
                    return 0; 
            }
        }
    }
}
