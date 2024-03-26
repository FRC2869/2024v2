// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.commands.setOdometry;
import frc.robot.generated.TunerConstants;

/**
 * Swerve drive.
 * @author Ankur "Rag(havan) to Riches"
 */
public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem instance;
  private CommandSwerveDrivetrain swerve;
  private ApplyChassisSpeeds speedsC;

  /**
   * Determines if the driver has control over swerve rotation.
   * @Values 1 = Driver control, 0 = Override
   */
  private static int rotationOverride = 1;

  public static SwerveSubsystem getInstance() {
    if (instance == null) instance = new SwerveSubsystem();
    return instance;
  }

  /** Creates a new Swerve. */
  public SwerveSubsystem() {
    swerve = TunerConstants.DriveTrain;
    speedsC = new ApplyChassisSpeeds();
    
    speedsC.withDriveRequestType(DriveRequestType.Velocity);
    speedsC.withSteerRequestType(SteerRequestType.MotionMagic);
    // SmartDashboard.putNumber("PosX", getPose().getX());
    // SmartDashboard.putNumber("PosY", getPose().getY());
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotRelativeSpeeds, 
      this::driveRobotRelative, 
      new HolonomicPathFollowerConfig(3, Math.sqrt(2)*20.75, 
      new ReplanningConfig()), 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      TunerConstants.DriveTrain
    );
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  //Find out how pose is supposed to be incorporated pls
  public void resetOdometry(Pose2d pose) {
  // swerve.tareEverything(); 
    swerve.seedFieldRelative(pose); 
  }

  /**
   * Gets the speeds of the chassis
   * @return a ChassisSpeeds which contains information on all the chassis
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveDriveKinematics kinematics = swerve.getKinematics();
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(swerve.getModule(0).getCurrentState(), swerve.getModule(1).getCurrentState(), swerve.getModule(2).getCurrentState(), swerve.getModule(3).getCurrentState());
    return speeds;
  }

  /**
   * Sets speed of robot
   * @param speeds ChassisSpeed to set to
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
  //   SwerveDriveKinematics kinematics = swerve.getKinematics();
  //   SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    speedsC.withSpeeds(speeds);
    //System.out.println(speeds);
    swerve.setControl(speedsC);
    // for (int i = 0; i < 4; i++)
    //   swerve.getModule(i).apply(states[i], DriveRequestType.Velocity);
  }

  /**
   * Rotation
   * @return rotation
   */
  public Rotation2d getHeading()
  {
    return swerve.getPigeon2().getRotation2d();
  }
  
  /**
   * @param pathName The name of your choreo path
   * @return A sequential command group created by Choreo Swerve Command 
   */
  public Command getTrajectory(String pathName) {
    PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(pathName);
    return new SequentialCommandGroup(new setOdometry(traj.getPreviewStartingHolonomicPose()), 
    AutoBuilder.followPath(traj));
  }
  /**
   * @param pathName Name of pathplanner path
   * @return A sequential command group created by Pathplanner swerve Command 
   */
  public Command getPathPlannerTrajectory(String pathName) {
    PathPlannerPath traj = PathPlannerPath.fromPathFile(pathName);
    return new SequentialCommandGroup(new setOdometry(traj.getPreviewStartingHolonomicPose()), 
    AutoBuilder.followPath(traj));
  }
  
  /**
   * Gets the autonomous command from Path Planner
   * @param autoName name of the autonomous
   * @return PathPlannerAuto to run
   */
  public Command getAuto(String autoName){
    
    resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));
    return new PathPlannerAuto(autoName);
  }

  /**
   * @return returns boolean which is true if communist
   */
  public boolean isRed(){
    var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
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

  /**
   * oH bOy I wOnDeR wHaT tHiS DoEs!!!!111!!???
   * @return command that will send robot to the source
   */
  public Command moveToPlayerStation() {
    List<Translation2d> list = PathPlannerPath.bezierFromPoses(getPose(), Constants.FieldConstants.humanPlayerStationPose);
    PathPlannerPath path = new PathPlannerPath(list, Constants.SwerveConstants.constraints, new GoalEndState(0, Constants.FieldConstants.humanPlayerStationPose.getRotation()));
    return AutoBuilder.followPath(path);
  }

  /**
   * Creates choreo trajectory based on positions
   * @param poses positions to get to
   * @return Choreo trajectory
   */
  public Command goToPosition(Pose2d... poses) {
    ArrayList<ChoreoTrajectoryState> list = new ArrayList<>();
    double time = 0;
    for (Pose2d pose : poses) {
      list.add(new ChoreoTrajectoryState(time, pose.getX(), pose.getY(), pose.getRotation().getRadians(), .5, .5, -.5));
      time += 10;
    }
    
    return getTrajectory(new ChoreoTrajectory(list));
  }
//Math.atan((getPose().getX())/getPose().getY())
  /**
   * Gives command that faces the speaker
   * @return command that will face the robot towards the speaker
   */
  public Command faceSpeaker() {
    double theta = Math.atan((getPose().getX())/getPose().getY());
    List<Translation2d> list = PathPlannerPath.bezierFromPoses(getPose(), new Pose2d(getPose().getTranslation(), new Rotation2d(theta)));
    PathPlannerPath path = new PathPlannerPath(list, Constants.SwerveConstants.constraints, new GoalEndState(0, new Rotation2d(theta)));
    return AutoBuilder.followPath(path);
  }

  public void switchOverrideState() {
    rotationOverride = Math.abs(rotationOverride - 1);
  }

  public static int getOverrideState() {
    return rotationOverride;
  }

  public Command moveToAmp() {
    List<Translation2d> list = PathPlannerPath.bezierFromPoses(getPose(), Constants.FieldConstants.ampLocation);
    PathPlannerPath path = new PathPlannerPath(list, Constants.SwerveConstants.constraints, new GoalEndState(0, Constants.FieldConstants.humanPlayerStationPose.getRotation()));
    return AutoBuilder.followPath(path);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Swerve Speed", getRobotRelativeSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve", TunerConstants.DriveTrain.getModule(0).getDriveMotor().getVelocity().getValue());
  }
  //Previous max speed of 4800 changed to 4000

  public double getTargetSlope(double x1, double y1, double x2, double y2) {
    double tAngle = Math.atan((y2-y1) / (x2-x1));
    double cAngle = getPose().getRotation().getDegrees();
    return (tAngle-cAngle)*.05;
  }
}