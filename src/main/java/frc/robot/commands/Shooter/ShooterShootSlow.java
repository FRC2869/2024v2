// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Revs the shooter to a slower rpm for beginning of auto
 * Instant Command
 */
public class ShooterShootSlow extends InstantCommand {
  private ShooterSubsystem shooter;

  public ShooterShootSlow() {
    shooter = ShooterSubsystem.getInstance();
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    // new LEDCommand(LightingSetting.AUTO).schedule();
    shooter.setSpeed(45, 45);
  }
}
