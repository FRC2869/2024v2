// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Loads the note into the shooter for amp scoring
 * Runs shooter with front and back in opposite directions so it doesn't go too far.
 * Instant Command 
 */
public class ShooterAmpLoad extends InstantCommand {
  private ShooterSubsystem shooter;

  public ShooterAmpLoad() {
    shooter = ShooterSubsystem.getInstance();
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    System.err.println("AMPL");
    shooter.setSpeed(15, -15);
  }
}
