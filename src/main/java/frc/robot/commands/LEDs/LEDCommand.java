// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants.LightingSetting;
import frc.robot.subsystems.LightingSubsystem;
/**
 * Controls the LEDs, construct with the enum that runs
 * Instant Command
 */ 
public class LEDCommand extends InstantCommand {
  LightingSubsystem lights = LightingSubsystem.getInstance();
  LightingSetting light;
  /** Creates a new ShootLEDCommand. */
  public LEDCommand(LightingSetting light) {
    addRequirements(lights);
    runsWhenDisabled();
    this.light = light;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lights.setLights(light);
  }
}
