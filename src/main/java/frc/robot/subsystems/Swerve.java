// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.commands.setOdometry;
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
    
    speedsC = new ApplyChassisSpeeds();
    
    speedsC.withDriveRequestType(DriveRequestType.Velocity);
    speedsC.withSteerRequestType(SteerRequestType.MotionMagic);
    SmartDashboard.putNumber("PosX", getPose().getX());
    SmartDashboard.putNumber("PosY", getPose().getY());
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
    //System.out.println(speeds);
    swerve.setControl(speedsC);
    // for (int i = 0; i < 4; i++)
    //   swerve.getModule(i).apply(states[i], DriveRequestType.Velocity);
  }

  public Rotation2d getHeading()
  {
    return swerve.getPigeon2().getRotation2d();
  }
  
  /**
   * @param pathName A human readable description of Path.
   * Paths include: 
   * @return A sequential command group created by Choreo Swerve Command 
   */
  public Command getTrajectory(String pathName) {
    ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
    
    return new SequentialCommandGroup(new setOdometry(traj.getInitialPose()), 
      Choreo.choreoSwerveCommand(
      traj, 
      this::getPose, 
      Choreo.choreoSwerveController(new PIDController(0, 0, 0), 
      new PIDController(0, 0, 0), new PIDController(0, 0, 0)), 
      this::driveRobotRelative, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          System.out.println(alliance.get());
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      TunerConstants.DriveTrain));
  }
  
  /**
   * @param pathName A human readable description of Path.
   * Paths include: 
   * @return A sequential command group created by Choreo Swerve Command 
   */
  public Command getTrajectory(ChoreoTrajectory traj) {
    return new SequentialCommandGroup(new setOdometry(traj.getInitialPose()), 
      Choreo.choreoSwerveCommand(
      traj, 
      this::getPose, 
      Choreo.choreoSwerveController(new PIDController(0, 0, 0), 
      new PIDController(0, 0, 0), new PIDController(0, 0, 0)), 
      this::driveRobotRelative, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          System.out.println(alliance.get());
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      TunerConstants.DriveTrain));
  }

  public Command moveToPlayerStation() {
    return goToPosition(getPose(), Constants.FieldConstants.humanPlayerStationPose);
  }

  public Command goToPosition(Pose2d... poses) {
    ArrayList<ChoreoTrajectoryState> list = new ArrayList<>();
    for (Pose2d pose : poses) {
      list.add(new ChoreoTrajectoryState(0, pose.getX(), pose.getY(), 0, .5, .5, .5));
    }
    return getTrajectory(new ChoreoTrajectory(list));
  }

  //Previous max speed of 4800 changed to 4000
}