// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeSpinStop extends InstantCommand {
  private IntakeSubsystem intake;

  public IntakeSpinStop() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = IntakeSubsystem.getInstance();
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.spinStop();
  }
}
