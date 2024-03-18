// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PositionsIntake;
import frc.robot.subsystems.IntakePivotSubsystem;
/**
 * Sets intake to base position
 * @author Dr. Kay
 */
public class IntakeBasePos extends Command {
  private IntakePivotSubsystem intakePivot;   

  /** Creates a new IntakeMove. */
  public IntakeBasePos() {
    intakePivot = IntakePivotSubsystem.getInstance();
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakePivot.setPositionControl(true);
    intakePivot.setPivotPos(IntakeConstants.basePosition);
    intakePivot.setCurrentPosition(PositionsIntake.BASE);
  }

  @Override
  public void end(boolean i){
    intakePivot.setPositionControl(false);
    intakePivot.setPivotSpeed(0);
  }
}
