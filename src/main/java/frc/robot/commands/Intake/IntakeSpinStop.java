// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSpinSubsystem;

/**
 * Stops the intake spinning
 * Instant Command
 */
public class IntakeSpinStop extends InstantCommand {
  private IntakeSpinSubsystem intake;

  public IntakeSpinStop() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = IntakeSpinSubsystem.getInstance();
    addRequirements(intake);
  }
boolean hasRun = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!hasRun){
      hasRun = true;
      System.out.println(this.getName()+ " Start:"+Constants.timer.get());
    }
    intake.spinStop();
  }
}
