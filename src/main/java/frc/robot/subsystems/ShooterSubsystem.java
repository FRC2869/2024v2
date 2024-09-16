// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.MotorConfiguration;

/**
 * Controls speed of shooter and measures if shooter
 * has been fully revved in autonomous.
 * @author on-core big raga(van) the opp stoppa
 */
public class ShooterSubsystem extends SubsystemBase {
  private SwerveSubsystem swerve;
  private PivotSubsystem pivot;
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
    swerve = SwerveSubsystem.getInstance();
    pivot = PivotSubsystem.getInstance();
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
  public double getRPS1() {
    return shooter1.getVelocity().getValueAsDouble();
  }
  public double getRPS2() {
    return shooter2.getVelocity().getValueAsDouble();
  }

  boolean atRPS1 = false;
  boolean atRPS2 = false;
  private boolean veloControl;
  /**
   * 
   * @return if the rps of the first shooter is more than the target rps, locks on until rps is 10 under target
   */
  public boolean isAtRPS() {
    if(getRPS1()-speed1>-15){
      atRPS1 = true;
    }else if(getRPS1()-speed1<-25){
      atRPS1 = false;
    }
    if(getRPS2()-speed2>-15){
      atRPS2 = true;
    }else if(getRPS1()-speed2<-25){
      atRPS2 = false;
    } 

    return atRPS1 && atRPS2 ;
  }

  public void setVeloControl(boolean isVeloControl){
    this.veloControl = isVeloControl;
  }

  public double getAngle() {
    return pivot.getAngle();
  }

  public double getChassisSpeed() {
    return Math.sqrt(Math.pow(swerve.getRobotRelativeSpeeds().vxMetersPerSecond, 2) + Math.pow(swerve.getRobotRelativeSpeeds().vyMetersPerSecond, 2));
  }

  public double getDistFromSpeaker() {
    return swerve.getDistanceFromSpeaker();
  }

  public double getSpeedOfNote() {
    return quadraticFormula(2 * 9.81 * Constants.FieldConstants.speakerHeight * Math.cos(getAngle())*Math.cos(getAngle()) - Math.sin(2*getAngle()), 4 * 9.81 * Constants.FieldConstants.speakerHeight * getChassisSpeed() * Math.cos(getAngle()) - 2 * getChassisSpeed() * Math.sin(getAngle()), 2 * 9.81 * Math.pow(getChassisSpeed(), 2) - Math.pow(swerve.getDistanceFromSpeaker()*9.81, 2));
  }

  public double quadraticFormula(double a, double b, double c) {
    return (-b + Math.sqrt(b*b - 4 * a * c))/(2 * a);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed1", getRPS1());
    SmartDashboard.putNumber("Shooter Speed2", shooter2.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Is at RPM", isAtRPS());
    SmartDashboard.putNumber("ShooterTargetSpeed", speed2);
    SmartDashboard.putNumber("Shooter % Power2", shooter2.get());
    SmartDashboard.putNumber("Shooter % Power1", shooter1.get());
    if(stopped){
      shooter1.stopMotor();
      shooter2.stopMotor();
    }else if(veloControl){
      var velo1 = new VelocityDutyCycle(speed1);
      var velo2 = new VelocityDutyCycle(speed2);
      shooter1.setControl(velo1);
      shooter2.setControl(velo2);
    }else{
      shooter1.set(speed1);
      shooter2.set(speed2);
      System.out.println(speed2);
    }
  
    // This method will be called once per scheduler run
  }
}
