package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 * @author The folks over at Rev 
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getPose() {
        return super.getState().Pose;
    }

    public SwerveDriveKinematics getKinematics() {
        return super.m_kinematics;
    }
    
    

    /**
     * @return angle between floor and speaker in degrees
     * MUST ADD Constants.PivotConstants.basePosition in set the encoder properly
     */
    public double getAngle() {
        return 180 * Math.atan(Constants.FieldConstants.speakerHeight/(Constants.FieldConstants.fieldWidth - Constants.FieldConstants.distanceFromWallSpeaker - getState().Pose.getX())) / Math.PI;
    }

    /**
     * Gets the angle between the intake and the shooter
     * intake angle is already added
     * @return
     */
    public double getIntakeAngle() {
        return 180*Math.atan((
            (Constants.PivotConstants.pivotRestingHeight + (Constants.PivotConstants.shooterPivotLength * Math.sin(Math.PI * (getAngle())/180)))
            /(Constants.PivotConstants.chassisWidth - (Constants.PivotConstants.shooterPivotLength * Math.cos(Math.PI * (getAngle())/180))))
             / Math.PI) - Constants.PivotConstants.intakeAngle;
    }
}
