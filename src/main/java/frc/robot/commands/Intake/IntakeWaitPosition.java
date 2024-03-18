// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;


/**
 * Waits for the intake to be at the right position
 * Doesn't add requirements
 * Ends on IntakePivotSubsystem.isAtPosition()
 */
public class IntakeWaitPosition extends Command {
  private IntakePivotSubsystem intake;

  /** Creates a new IntakeWaitPosition. */
  public IntakeWaitPosition() {
    intake = IntakePivotSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake.isAtPosition()){
      return true;
    }
    return false;
  }
}
