// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

/**
 * sets the odometry to the given Pose2d
 */
public class setOdometry extends InstantCommand {
  private Swerve swerve;
  private Pose2d pose;

  public setOdometry(Pose2d pose) {
    swerve = Swerve.getInstance();
    this.pose = pose;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    swerve.resetOdometry(pose);
  }
}
