// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShooterAmpLoad;
import frc.robot.commands.ShooterAmpScore;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterStop;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Inputs.getShooterShoot().onTrue(new ShooterShoot());
    Inputs.getShooterStop().onTrue(new ShooterStop());
    Inputs.getShooterAmpLoad().onTrue(new ShooterAmpLoad());
    Inputs.getShooterAmpScore().onTrue(new ShooterAmpScore());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
