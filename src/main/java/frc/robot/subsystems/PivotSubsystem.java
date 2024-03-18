// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.MotorConfiguration;

/**
<<<<<<< Updated upstream
 * Controls the Shooter Pivot
=======
 * Controls the pivot of the shooter.
 * @author VP
>>>>>>> Stashed changes
 */
public class PivotSubsystem extends SubsystemBase {
  public static PivotSubsystem instance;
  /**
     * Creates a singleton instance of the PivotSubsystem
     * @return PivotSubsystem instance
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
<<<<<<< Updated upstream
	 * Gets the current angle of the pivot
	 * @return
=======
	 * 
	 * @return the pivot angle in revolutions
>>>>>>> Stashed changes
	 */
	public double getAngle() {
		// return -collection.getIntegratedSensorPosition();
		return pivotMotor.getEncoder().getPosition();
	}

	/**
     * Checks if the pivot is at the position
     * if going to base, will be true 3 before the position
     * if going to other positon will be +- .5
     * @return true if at position
     */
	public boolean isAtPosition(){
		if(currentPos != PositionsPivot.BASE)
			return Math.abs(pos-getAngle())<.5;
		else 
			return pos<PivotConstants.basePosition-3;
	}

	/**
     * increases target angle by 2
     */
	public void adjustUp() {
		pos += 2;
	}

	/**
     * decreases target angle by 2
     */
	public void adjustDown() {
		pos -= 2;
	}

	/**
     * Sets the current position enum
     * @param pos PositionsPivot of the target position
     */
	public void setCurrentPosition(PositionsPivot pos) {
		currentPos = pos;
	}

	/**
     * Sets the target speed of the pivot in percent output
     * Only applies when isPosControl == false
        * @param speed the speed of the pivot [-1, 1]
     */
	public void setSpeed(double speed) {
        this.speed = speed;
    }

	/**
     * Sets whether or not to use position control on the pivot
     * @param isPosControl true = position, false = speed
     */
	public void setPositionControl(boolean isPosControl){
		this.isPosControl = isPosControl;
	}

	private boolean isPosControl;
	private double speed;

	@Override
  public void periodic() {
	if (isPosControl) {
		if(currentPos!=PositionsPivot.BASE||getAngle()<PivotConstants.basePosition){
			SmartDashboard.putBoolean("enabled", true);
			pivotMotor.getPIDController().setReference(pos, ControlType.kPosition);
		}else{
			pivotMotor.set(0);	
			SmartDashboard.putBoolean("enabled", false);

		}
	}else{
		pivotMotor.set(speed);
	}
	
	SmartDashboard.putNumber("Angle Shooter", getAngle());
	SmartDashboard.putBoolean("Pivot At Pos", isAtPosition());
  }

  	/**
     * toggles brake mode on the motor
     */
  	public void toggleBrake(){
        if(pivotMotor.getIdleMode()==IdleMode.kCoast)
            pivotMotor.setIdleMode(IdleMode.kBrake);
        else
            pivotMotor.setIdleMode(IdleMode.kCoast);
    }

	/**
     * sets the motor to brake mode
     */
    public void setBrake(){
        pivotMotor.setIdleMode(IdleMode.kBrake);
    }
}

