// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
  private static Swerve instance;
  private CommandSwerveDrivetrain swerve;
  private ApplyChassisSpeeds speedsC;

  public static Swerve getInstance() {
    if (instance == null) instance = new Swerve();
    return instance;
  }

  /** Creates a new Swerve. */
  public Swerve() {
    swerve = TunerConstants.DriveTrain;
    setupPathPlanner();
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  //Find out how pose is supposed to be incorporated pls
  public void resetOdometry(Pose2d pose) {
  // swerve.tareEverything(); 
  swerve.seedFieldRelative(pose); 
}

  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveDriveKinematics kinematics = swerve.getKinematics();
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(swerve.getModule(0).getCurrentState(), swerve.getModule(1).getCurrentState(), swerve.getModule(2).getCurrentState(), swerve.getModule(3).getCurrentState());
    return speeds;
  }
  public void driveRobotRelative(ChassisSpeeds speeds) {
  //   SwerveDriveKinematics kinematics = swerve.getKinematics();
  //   SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    speedsC.withSpeeds(speeds);
    System.out.println(speeds);
    swerve.setControl(speedsC);
    // for (int i = 0; i < 4; i++)
    //   swerve.getModule(i).apply(states[i], DriveRequestType.Velocity);
  }

  public Rotation2d getHeading()
  {
    return swerve.getPigeon2().getRotation2d();
  }

  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         new PIDConstants(1.7, 0.0, 0.65),
                                         // Translation PID constants
                                         new PIDConstants(0.118,
                                                          0,
                                                          0.3),
                                         // Rotation PID constants
                                         4.5,
                                         // Max module speed, in m/s
                                         14.67,
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        TunerConstants.DriveTrain // Reference to this subsystem to set requirements
                                  );

       speedsC = new ApplyChassisSpeeds();
  speedsC.withDriveRequestType(DriveRequestType.Velocity);
  speedsC.withSteerRequestType(SteerRequestType.MotionMagic);

  }

  public Command getPathCommand(String pathName, boolean setOdomToStart) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    if (setOdomToStart)
    {
      resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    
    return AutoBuilder.followPath(path);
  }
}
