// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.MotorConfiguration;

/**
 * Controls speed of shooter and measures if shooter
 * has been fully revved in autonomous.
 * @author on-core big raga(van) the opp stoppa
 */
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
    MotorConfiguration.configureMotor(shooter2, ShooterConstants.config2);
  }

  /**
   * Sets the speed in rotations per second of the motor
   * @param speed1 speed of motor close to intake
   * @param speed2 speed of motor far from intake
   */
  public void setSpeed(double speed1, double speed2){
    this.speed1 = speed1;
    this.speed2 = speed2;
    System.out.println(speed1+" "+speed2);
    stopped = false;
  }

  /**
   * stops both shooter motors
   */
  public void stop(){
    this.speed1 = 0;
    this.speed2 = 0;
    stopped = true;
    
  }

  /**
   * 
   * @return rotations/second of the first shooter 
   */
  public double getRPS() {
    return shooter1.getVelocity().getValueAsDouble();
  }

  boolean atRPS = false;
  /**
   * 
   * @return if the rps of the first shooter is more than the target rps, locks on until rps is 10 under target
   */
  public boolean isAtRPS() {
    if(getRPS()-speed1>-10){
      atRPS = true;
    }else if(getRPS()-speed1<-20){
      atRPS = false;
    }
    return atRPS;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed1", getRPS());
    SmartDashboard.putNumber("Shooter Speed2", shooter2.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Is at RPM", isAtRPS());
    SmartDashboard.putNumber("ShooterTargetSpeed", speed2);
    SmartDashboard.putNumber("Shooter % Power", shooter2.get());
    if(stopped){
      shooter1.stopMotor();
      shooter2.stopMotor();
    }else{
      var velo1 = new VelocityDutyCycle(speed1);
      var velo2 = new VelocityDutyCycle(speed2);
      shooter1.setControl(velo1);
      shooter2.setControl(velo2);
    }
  
    // This method will be called once per scheduler run
  }
}
