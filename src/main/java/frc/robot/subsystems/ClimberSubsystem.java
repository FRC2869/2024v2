// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

// import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * this is where i would put the code for the climber
 * IF I HAD ONE
 * @author Tong Boyle (8/10 functional fingers)
 */
public class ClimberSubsystem extends SubsystemBase {

  private static ClimberSubsystem instance;
  private TalonFX talon;

  /**
   * @return An instance of ClimberSubsystem
   */
  public static ClimberSubsystem getInstance() {
    if (instance == null) instance = new ClimberSubsystem();
    return instance;
  }
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    talon = new TalonFX(21);
  }

  /**
   * Sets speed of climber
   * @param speed speed (-1 to 1) in percent
   */
  public void set(double speed) {
    talon.set(speed);
  }
}
