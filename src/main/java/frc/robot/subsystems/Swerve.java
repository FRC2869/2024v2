// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
  private Swerve instance;
  private CommandSwerveDrivetrain swerve;

  public Swerve getInstance() {
    if (instance == null) instance = new Swerve();
    return instance;
  }

  /** Creates a new Swerve. */
  public Swerve() {
    swerve = TunerConstants.DriveTrain;
    AutoBuilder.configureHolonomic(
      this::getPose, //pose2d
      this::resetPose, //pose2d
      this::getRobotRelativeSpeeds, //ChassisSpeeds
      this::driveRobotRelative, //ChassisSpeeds
      new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),
        4.5,
        0.4,
        new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  //Find out how pose is supposed to be incorporated pls
  public void resetPose(Pose2d pose) {
    swerve.tareEverything();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveDriveKinematics kinematics = swerve.getKinematics();
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(swerve.getModule(0).getCurrentState(), swerve.getModule(1).getCurrentState(), swerve.getModule(2).getCurrentState(), swerve.getModule(3).getCurrentState());
    return speeds;
  }
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveDriveKinematics kinematics = swerve.getKinematics();
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    for (int i = 0; i < 4; i++)
      swerve.getModule(4).apply(states[0], null);
  }

  public Command getPathCommand(String path) {
    PathPlannerPath pathCommand = PathPlannerPath.fromPathFile(path);
    return AutoBuilder.followPath(pathCommand);
  }
}
