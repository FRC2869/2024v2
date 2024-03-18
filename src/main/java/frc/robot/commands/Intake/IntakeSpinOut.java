// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Spins the intake to outtake the note
 * Instant Command
 */
public class IntakeSpinOut extends InstantCommand {
  private IntakeSubsystem intake;

  public IntakeSpinOut() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = IntakeSubsystem.getInstance();
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.spinOut();
  }
}
