package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class Constants {
    public static Timer timer = new Timer();

    public static class FieldConstants {
        //ALL ARE SET TO INCHES
        public static final double fieldWidth = 323.125;

        public static final double speakerHeight = 78;

        public static final double distanceFromWallSpeaker = 9;
        
        //THIS NEEDS SET!!!!!!
        public static final Pose2d humanPlayerStationPose = new Pose2d(15.326556205749512, 1.4545217752456665, new Rotation2d(-56.309925735));
        //except this, this is in meters!
        /**
         * meters
         */
        public static final double speakerToOriginDist = 291;

    }

    public static class OperatorConstants {
        public static interface DriverController {
            public static final int port = 0;
            public static final double leftXDeadband = .01;
            public static final double leftYDeadband = .01;
            public static final double rightXDeadband = .01;
            public static final double righYDeadband = .01;
        }

        public static interface DriverController2 {
            public static final int port = 1;
            public static final double leftXDeadband = .01;
            public static final double leftYDeadband = .01;
            public static final double rightXDeadband = .01;
            public static final double righYDeadband = .01;
        }

        public static interface OperatorController {
            public static final int port = 2;
            public static final double leftXDeadband = .01;
            public static final double leftYDeadband = .01;
            public static final double rightXDeadband = .01;
            public static final double righYDeadband = .01;
        }

        public static interface OperatorButtonBoard {
            public static final int port = 3;
        }
    }

    public static class ShooterConstants {
        public static enum LightingSetting {
            TELEOP, ORANGEWHITEGREENWITHBLUESPECKSITUATION, CANSHOOT, AUTO, DISABLED, GAME, LOSE, WIN, Pattern1, Pattern2, Pattern3, Pattern4
          }
        public static final int id2 = 18;
        public static final int id1 = 19;
        public static final MotorConfiguration config = new MotorConfiguration(1.0, 0, 0, 0, 0, 1, -1, 0, 0, 40,
                true, false, 0);
    }

    public static class IntakeConstants {
        public static final MotorConfiguration spinMotorConfig = new MotorConfiguration(1, -1, 40, true, false);
        public static final MotorConfiguration pivotMotorConfig = new MotorConfiguration(1.0, 0, 10, 0, 0, 0.6, -0.6,
                (5 * (50 / 16.0) * (32 / 16.0)), 0, 40, false, false, 0);
        public static final double kMinAngle = -770;
        public static final double kMaxAngle = 0;
        public static final double floorPosition = -765;
        public static final double basePosition = 0;
        public static final double closePosition = -50;
        public static final double farPosition = -30;//-30

        public static enum PositionsIntake {
            BASE, FLOOR, FAR, CLOSE
        }

        public static double getTargetPos(PositionsIntake intakeEnum) {
            switch (intakeEnum) {
                case BASE:
                    return IntakeConstants.basePosition;
                case FLOOR:
                    return IntakeConstants.floorPosition;
                default:
                    return 0;
            }
        }
    }

    public static class PivotConstants {
        public static final int id = 17;
        public static final MotorConfiguration config = new MotorConfiguration(1.0, 0, 2, 0, 0, .25, -.5, ((16/30.0)*(48.0/48.0)*(25.0)), 0, 40, false, false, 0);
        public static final int basePosition = -30;
        public static final double kMinAngle = -200;
        public static final double kMaxAngle = 0;
        public static final double ampPosition = -170;//-170
        public static final double farPosition = -20;//-22
        public static final double closePosition = 0;//-7
        /**inches */
        public static final double pivotRestingHeight = 25; // not good
        /**inches */
        public static final double chassisWidth = 26; // not good
        /**inches */
        public static final double shooterPivotLength = 9; // not good
        // public static final double intakeAngle = 180 - 140;
        /**degrees */
        public static final double intakeAngle = 67;
        public static final double startingAngle = 30;
        public static enum PositionsPivot{
            BASE, AMP, FAR, CLOSE,
        } //57, 13
        public static double adjustment = 2;
    }
}
