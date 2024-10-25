// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Revs the shooter to shoot to the speaker
 * Instant Command
 */
public class ShooterIntake extends InstantCommand {
  private ShooterSubsystem shooter;

  public ShooterIntake() {
    shooter = ShooterSubsystem.getInstance();
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }
boolean hasRun = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!hasRun){
      hasRun = true;
      System.out.println(this.getName()+ " Start:"+Constants.timer.get());
    }
    shooter.setSpeed(-.5, -.5);
    shooter.setVeloControl(false);
    // new LEDCommand(LightingSetting.AUTO).schedule();
  }
}
