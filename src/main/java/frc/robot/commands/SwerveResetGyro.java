// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Resets the pose of the robot to the same position but 0 rotation
 */
public class SwerveResetGyro extends InstantCommand {
  private SwerveSubsystem swerve;

  public SwerveResetGyro() {
    swerve = SwerveSubsystem.getInstance();
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    swerve.resetOdometry(new Pose2d(swerve.getPose().getTranslation(), new Rotation2d()));;
  }
}
