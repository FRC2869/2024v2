// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Waits for the Shooter to spin to the right speed
 * Doesn't add requirements
 * Ends on Shooter +- 2 rps
 */
public class ShooterRevWait extends Command {
  /** Creates a new ShooterRevWait. */
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public ShooterRevWait() {
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.isAtRPS());
  }
}
