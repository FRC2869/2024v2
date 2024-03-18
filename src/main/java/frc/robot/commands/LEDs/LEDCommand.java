// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants.LightingSetting;
import frc.robot.subsystems.LightingSubsystem;

/**
 * Sets the LED to a certain mode
 * @author Umesh Poragupati
 */
public class LEDCommand extends InstantCommand {
  private LightingSubsystem lights = LightingSubsystem.getInstance();
  private LightingSetting light;
  /** Creates a new ShootLEDCommand. */
    
  /**
   * Sets the LED to a certain mode
   * @param light the LightSetting that the lights will be changed to.
   */
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
