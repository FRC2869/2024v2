// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.MotorConfiguration;

/**
 * Controls the pivot of the shooter.
 * @author VP
 */
public class PivotSubsystem extends SubsystemBase {
  public static PivotSubsystem instance;

  /**
   * PivotSubsystems rather infamous get instance function
   * @returns PivotSubsystem instance
   */
  public static PivotSubsystem getInstance(){
    if (instance==null) instance = new PivotSubsystem();
    return instance;
  }

  private CANSparkMax pivotMotor;
  private double pos;
  private PositionsPivot currentPos; 

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    pivotMotor = new CANSparkMax(PivotConstants.id, MotorType.kBrushless);
    MotorConfiguration.configureMotor(pivotMotor, PivotConstants.config);
  }

/**
	 * Sets the pivot motor to the specified position
	 * Only applies if isPosControl == true
	 * 
	 * @param pos the position to set the motor to [kMinAngle, kMaxAngle]
	 */
	public void position(double pos) {
		this.pos = MathUtil.clamp(pos, PivotConstants.kMinAngle, PivotConstants.kMaxAngle);
		// this.pos = pos;
	}

	/**
	 * @return the pivot angle in revolutions
	 */
	public double getAngle() {
		// return -collection.getIntegratedSensorPosition();
		return pivotMotor.getEncoder().getPosition();
	}

	/**
	 * @return boolean which is true when at the position it is trying to go
	 */
	public boolean isAtPosition(){
		if(currentPos != PositionsPivot.BASE)
			return Math.abs(pos-getAngle())<.5;
		else 
			return pos<PivotConstants.basePosition;
	}

	/**
	 * moves pivot up Constants.PivotConstants.adjustment
	 */
	public void adjustUp() {
		pos += Constants.PivotConstants.adjustment;
	}

	/**
	 * moves pivot down Constants.PivotConstants.adjustment
	 */
	public void adjustDown() {
		pos -= Constants.PivotConstants.adjustment;
	}

	/**
	 * 
	 * @param pos sets the position it is trying to go at.
	 */
	public void setCurrentPosition(PositionsPivot pos) {
		currentPos = pos;
	}

	/**
	 * sets speed
	 * @param speed percent
	 */
	public void setSpeed(double speed) {
        this.speed = speed;
    }

	/**
	 * set the ability to move
	 * @param isPosControl true when able to move
	 */
	public void setPositionControl(boolean isPosControl){
		this.isPosControl = isPosControl;
	}

	// private DecimalFormat rounder = new DecimalFormat("#.0");
	private boolean isPosControl;
	private double speed;

	@Override
  public void periodic() {
	// var angleString = rounder.format(getAngle());
	// SmartDashboard.putString("Pivot Angle 1", angleString);
	// SmartDashboard.putBoolean("pivotPos", isPosControl);
	// SmartDashboard.putNumber("pos", pos);
	if (isPosControl) {
		if(currentPos!=PositionsPivot.BASE||getAngle()<PivotConstants.basePosition){
			// SmartDashboard.putBoolean("enabled", true);
			pivotMotor.getPIDController().setReference(pos, ControlType.kPosition);
		}else{
			pivotMotor.set(0);	
			// SmartDashboard.putBoolean("enabled", false);

		}
	}else{
		pivotMotor.set(speed);
	}
	//lRDXFGRER
	
	SmartDashboard.putNumber("Angle Shooter", getAngle());
	// SmartDashboard.putBoolean("Pivot At Pos", isAtPosition());
  }

  /**
   * Brakes
   */
  public void toggleBrake(){
        if(pivotMotor.getIdleMode()==IdleMode.kCoast)
            pivotMotor.setIdleMode(IdleMode.kBrake);
        else
            pivotMotor.setIdleMode(IdleMode.kCoast);
    }

	/**
	 * Sets brakes
	 */
    public void setBrake(){
        pivotMotor.setIdleMode(IdleMode.kBrake);
    }

	public void autoAim(double distance) {
		double theta = Math.atan((PivotConstants.speakerHeight - PivotConstants.shooterHeight) / (distance));
		position(PivotConstants.basePosition + theta);
		isPosControl = true;
	}
}

