// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSpinSubsystem;
/**
 * Waits for the intake to stop spinning
 * Doesn't add requirements
 * Ends on Intake +- 3 rps
 */
public class IntakeWaitNote extends Command {
  private IntakeSpinSubsystem intake;
  /** Creates a new IntakeWaitNote. */
  public IntakeWaitNote() {
    intake = IntakeSpinSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
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
      System.out.println(this.getName()+ " Start:"+Constants.timer.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName()+ " End:"+Constants.timer.get());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return intake.isIntake()||timer.get()>2;
    return intake.isIntake();
  }
}
