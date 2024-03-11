// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfiguration;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  public static ShooterSubsystem instance;
  public static ShooterSubsystem getInstance(){
    if(instance == null) instance = new ShooterSubsystem();
    return instance;
  }

  private TalonFX shooter1;
  private TalonFX shooter2;
  private double speed1;
  private double speed2;
  private boolean stopped = true;
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooter1 = new TalonFX(ShooterConstants.id1);
    shooter2 = new TalonFX(ShooterConstants.id2);
    MotorConfiguration.configureMotor(shooter1, ShooterConstants.config);
    MotorConfiguration.configureMotor(shooter2, ShooterConstants.config);
  }

  public void setSpeed(double speed1, double speed2){
    this.speed1 = speed1;
    this.speed2 = speed2;
    stopped = false;
  }

  public void stop(){
    this.speed1 = 0;
    this.speed2 = 0;
    stopped = true;
    
  }

  @Override
  public void periodic() {
    // var velo1 = new Diff_VelocityDutyCycle_Velocity(new VelocityDutyCycle(speed1, 3.0, true, 0, 0, false, false, false), new VelocityDutyCycle(speed1));
    // var velo2 = new Diff_VelocityDutyCycle_Velocity(new VelocityDutyCycle(speed2, 3.0, true, 0, 0, false, false, false), new VelocityDutyCycle(speed2));
    if(stopped){
      shooter1.stopMotor();
      shooter2.stopMotor();
    }else{
      var velo1 = new VelocityDutyCycle(speed1);
      var velo2 = new VelocityDutyCycle(speed2);
      shooter1.setControl(velo1);
      
      
      shooter2.setControl(velo2);
      // shooter1.set(speed1);
      // shooter2.set(speed2);
    }
    // This method will be called once per scheduler run
  }
}
