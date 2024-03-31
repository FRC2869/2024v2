// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LightingState;

public class LimelightLightingDefault extends Command {
  private LimelightSubsystem limelight;

  /** Creates a new LimelightLightingDefault. */
  public LimelightLightingDefault() {
    limelight = LimelightSubsystem.getInstance();
    addRequirements(limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.getCurrentLightingState()!=LightingState.OFF&&Timer.getMatchTime()<1){
      limelight.setLEDsOff();
    }else if(limelight.getCurrentLightingState()!=LightingState.ON&&Timer.getMatchTime()<21){
      limelight.setLEDsOn();
    }else if(limelight.getCurrentLightingState()!=LightingState.BLINK&&Timer.getMatchTime()<30){
      limelight.setLEDsBlink();
    }else if(limelight.getCurrentLightingState()!=LightingState.OFF){
      limelight.setLEDsOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
