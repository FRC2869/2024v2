// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PositionsIntake;
import frc.robot.subsystems.IntakePivotSubsystem;

/**
 * Sets the intake to the Far Position
 * Never Ends
 * On Interupt switches to speed control and sets speed to 0
 */
public class IntakeLobPos extends Command {
  private IntakePivotSubsystem intakePivot;

  /** Creates a new IntakeMove. */
  public IntakeLobPos() {
    intakePivot = IntakePivotSubsystem.getInstance();
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  boolean hasRun = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!hasRun){
      hasRun = true;
      System.out.println("IntakeFarPos Start:"+Constants.timer.get());
    }
    intakePivot.setPositionControl(true);
    intakePivot.setPivotPos(IntakeConstants.lobPosition);
    intakePivot.setCurrentPosition(PositionsIntake.LOB);
  }

  @Override
  public void end(boolean i){
    System.out.println("IntakeLobPos End:"+Constants.timer.get());
    intakePivot.setPositionControl(false);
    intakePivot.setPivotSpeed(0);
  }
}
