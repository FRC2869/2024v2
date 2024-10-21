// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSpinSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeFromSource extends Command {
  private IntakeSpinSubsystem intake;
  private ShooterSubsystem shooter;
  /** Creates a new IntakeFromSource. */
  public IntakeFromSource() {
    intake = IntakeSpinSubsystem.getInstance();
    shooter = ShooterSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.spinIn();
    shooter.setSpeed(-.5, -.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.spinStop();
    shooter.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
