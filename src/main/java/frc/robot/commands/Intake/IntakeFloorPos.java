// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PositionsIntake;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakeFloorPos extends Command {
  private IntakePivotSubsystem intakePivot;
  private boolean hasRun;

  /** Creates a new IntakeMove. */
  public IntakeFloorPos() {
    intakePivot = IntakePivotSubsystem.getInstance();
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasRun = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!hasRun){
    intakePivot.setPositionControl(true);
    intakePivot.setPivotPos(IntakeConstants.floorPosition);
    intakePivot.setCurrentPosition(PositionsIntake.FLOOR);
    hasRun = true;
    }
  }

  @Override
  public void end(boolean i){
    intakePivot.setPositionControl(false);
    intakePivot.setPivotSpeed(0);
  }
}
