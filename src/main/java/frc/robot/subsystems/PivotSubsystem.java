// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import java.text.DecimalFormat;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfiguration;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;

public class PivotSubsystem extends SubsystemBase {
  public static PivotSubsystem instance;
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

	public double getAngle() {
		// return -collection.getIntegratedSensorPosition();
		return pivotMotor.getEncoder().getPosition();
	}

	public boolean isAtPosition(){
		if(currentPos != PositionsPivot.BASE)
			return Math.abs(pos-getAngle())<.5;
		else 
			return pos<PivotConstants.basePosition;
	}

	public void adjustUp() {
		pos += 2;
	}

	public void adjustDown() {
		pos -= 2;
	}

	public void setCurrentPosition(PositionsPivot pos) {
		currentPos = pos;
	}

	public void setSpeed(double speed) {
        this.speed = speed;
    }


	public void setPositionControl(boolean isPosControl){
		this.isPosControl = isPosControl;
	}

	private DecimalFormat rounder = new DecimalFormat("#.0");
	private boolean isPosControl;
	private double speed;

	@Override
  public void periodic() {
	var angleString = rounder.format(getAngle());
	SmartDashboard.putString("Pivot Angle 1", angleString);
	if (isPosControl) {
		if(true)
			pivotMotor.getPIDController().setReference(pos, ControlType.kPosition);
		else
			pivotMotor.set(0);	
	}else{
		pivotMotor.set(speed);
	}
  }

  public void toggleBrake(){
        if(pivotMotor.getIdleMode()==IdleMode.kCoast)
            pivotMotor.setIdleMode(IdleMode.kBrake);
        else
            pivotMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake(){
        pivotMotor.setIdleMode(IdleMode.kBrake);
    }
}
