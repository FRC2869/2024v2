// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Scores the note into the AMP
 * Runs the Shooter at low power
 * Instant Command
 */
public class ShooterAmpScore extends InstantCommand {
  private ShooterSubsystem shooter;

  public ShooterAmpScore() {
    shooter = ShooterSubsystem.getInstance();
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(15, -15);
  }
}
