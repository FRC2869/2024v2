package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class Constants {
    public static Timer timer = new Timer();
    public enum RobotState {
        DISABLED, TELEOP, AUTON, TEST
    }
    public static RobotState currentRobotState = RobotState.DISABLED;

    public static class FieldConstants {

        public static final Pose2d redSpeakerPose = new Pose2d(16.36, 5.528, new Rotation2d(0));
        public static final Pose2d blueSpeakerPose = new Pose2d(.2, 5.528, new Rotation2d(0));
        //ALL ARE SET TO INCHES
        
        /** meters */
        public static final double fieldWidth = 323.125/39.37;
        /**meters */
        public static final double speakerHeight = 78/39.37;

        /**meters */
        public static final double distanceFromWallSpeaker = 9/39.37;
        
        /**meters */
        public static final double distYSpeaker = 5.56;
        
        
        public static final Pose2d humanPlayerStationPose = new Pose2d(15.326556205749512, 1.4545217752456665, new Rotation2d(-56.309925735));

        
        public static final Pose2d ampLocation = new Pose2d(1.4101016521453857, 7.223808288574219, new Rotation2d(Math.PI/2));
        //except this, this is in meters!

        /** meters */
        public static final double speakerToOriginDist = 291;

    }

    public static class OperatorConstants {
        //was 7.5 before
        public static final double deadBandMove = .15;
        public static final double deadBandRot = .15;
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
            TELEOP, ORANGEWHITEGREENWITHBLUESPECKSITUATION, CANSHOOT, AUTO, DISABLED, GAME, LOSE, WIN, Pattern1, Pattern2, Pattern3, Pattern4, GAMEOVER, INTAKING
          }
        public static final int id2 = 18;
        public static final int id1 = 19;
        public static final MotorConfiguration config = new MotorConfiguration(0.075, 0, 0, 0, 2, 1, -1, 0, 0, 60,
                true, false, 0);
        public static final MotorConfiguration config2 = new MotorConfiguration(0.075, 0, 0, 0, 0, 1, -1, 0, 0, 40,
                true, true, 0);
    }

    public static class IntakeConstants {
        public static final MotorConfiguration spinMotorConfig = new MotorConfiguration(1, -1, 40, true, false);
        public static final MotorConfiguration pivotMotorConfig = new MotorConfiguration(1.0, 0, 10, 0, 0, 0.6, -0.6,
                (5 * (50 / 16.0) * (32 / 16.0)), 0, 60, false, false, 0);
        public static final double kMinAngle = -750;
        public static final double kMaxAngle = 0;
        // public static final double floorPosition = -710;
        // public static final double floorPosition = -650;
        public static final double floorPosition = -725;
        public static final double basePosition = 0;
        public static final double closePosition = -17;
        public static final double farPosition = -80;//-45
        public static final double lobPosition = -60;//-30

        public static enum PositionsIntake {
            BASE, FLOOR, FAR, CLOSE, LOB
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
        public static final double intakeGearMultiplier = 3 * 3 * 2 * 360 * 32 / 18.0;
        
        public static final int id = 17;
        public static final MotorConfiguration config = new MotorConfiguration(1.0, 0, 5, 0, -3, .3, -.5, ((18.0/30.0)*(1.0/25.0))*360, 38, 40, false, false, 0);
        public static final int basePosition = 30;//-30
        public static final double kMinAngle = -70;
        public static final double kMaxAngle = 38;
        public static final double ampPosition = -50;//-50
        public static final double farPosition = 5;//-35
        public static final double lobPosition = 5;//-22
        public static final double closePosition = 38;//-7
        public static final double climbPosition = -70;
        /**inches */
        public static final double pivotRestingHeight = 25; // not good
        /**inches */
        public static final double chassisWidth = 26; // not good
        /**inches */
        public static final double shooterPivotLength = 9; // not good
        // public static final double intakeAngle = 180 - 140;
        /**degrees */
        public static final double intakeAngle = 67;
        public static final double startingAngle = 38;
        public static final double shooterHeight = 23.4;
        public static enum PositionsPivot{
            BASE, AMP, FAR, CLOSE, CLIMB,
        } //57, 13
        public static double adjustment = 2;
        public static double smallAdjustment = .5;

        public static final double blueSpeakerX = 0.365;
        public static final double blueSpeakerY= 5.56;
        public static final double redSpeakerX = 16.154;
        public static final double redSpeakerY = 5.56;

        public static final double speakerHeight = 2.05;
    }

    public static class SwerveConstants {
        public static PathConstraints constraints = new PathConstraints(3.783, 5.380, 10.151, 24.760);
    }
}

// blue speaker
// x = 15 in or 0.365 m
// y = 218 in or 5.56 m

// red speaker
// x = 635 in or 16.155 m
// y = 218 in or 5.56 m
