package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PositionsIntake;
import frc.robot.Constants;
import frc.robot.MotorConfiguration;

/**
 * Controls the pivot of the intake.
 * @author "Arsh, the Ankur is here" -Mirage
 */
public class IntakePivotSubsystem extends SubsystemBase{
    private static IntakePivotSubsystem instance;
    
    /**
     * Creates a singleton instance of the IntakePivotSubsytem
     * @return IntakePivotSubsystem instance
     */
    public static IntakePivotSubsystem getInstance(){
        if (instance == null) instance = new IntakePivotSubsystem();
        return instance;
    }

    private CANSparkFlex pivotMotor;
    private boolean isPosControl = false;
    private double pivotPos;
    private double pivotSpeed;
    private PositionsIntake currentPos = PositionsIntake.BASE;
    private RelativeEncoder encoder;

    public IntakePivotSubsystem() {
        pivotMotor = new CANSparkFlex(16, MotorType.kBrushless);
        encoder = MotorConfiguration.configureMotor(pivotMotor, IntakeConstants.pivotMotorConfig);
    }

    /**
     * Sets the target position of the pivot
     * Only applies when isPosControl == true
     * @param pos the position you would like to move the intake to [IntakeConstants.kMinAngle, IntakeConstants.kMaxAngle]
     */
    public void setPivotPos(double pos){
        // this.pivotPos = MathUtil.clamp(pos, IntakeConstants.kMinAngle, IntakeConstants.kMaxAngle);
        this.pivotPos = pos; 
    }

    /**
     * Sets the target speed of the pivot in percent output
     * Only applies when isPosControl == false
        * @param speed the speed of the pivot [-1, 1]
     */
    public void setPivotSpeed(double speed){
        this.pivotSpeed = speed;
    }

    /**
     * Resets the pivot position to IntakeConstants.basePosition
     */
    public void resetPivot() {
        encoder.setPosition(IntakeConstants.basePosition);
    }

    /**
     *  
     * @return angle of the intake pivot
     */
    public double getAngle(){
        return encoder.getPosition();
    }

    /**
     * Sets whether or not to use position control on the pivot
     * @param isPosControl true = position, false = speed
     */
    public void setPositionControl(boolean isPosControl){

		this.isPosControl = isPosControl;

	}

    /**
     * Checks if the pivot is at the position
     * if going to base or floor, will be true 5 before the position
     * if going to other positon will be +- 3
     * @return true if at position
     */
    public boolean isAtPosition(){
		if(currentPos == PositionsIntake.BASE){
            return getAngle()>=IntakeConstants.basePosition-5;
        }else if(currentPos == PositionsIntake.FLOOR){
            return getAngle()<=IntakeConstants.floorPosition+5;
        }else {
            return Math.abs(pivotPos - getAngle()) < 3;
        }
			
	}

    /**
     * increases target angle by Constants.PivotConstants.adjustment
     */
    public void adjustUp() {
		pivotPos += Constants.PivotConstants.adjustment;
	}

    /**
     * decreases target angle by Constants.PivotConstants.adjustment
     */
	public void adjustDown() {
		pivotPos -= Constants.PivotConstants.adjustment;
	}

    /**
     * Sets the current position enum
     * @param pos PositionsIntake of the target position
     */
    public void setCurrentPosition(PositionsIntake pos) {
		currentPos = pos;
	}

    @Override
    public void periodic(){
        SmartDashboard.putNumber("intake", getAngle());
        if (isPosControl) {
            if((currentPos!=PositionsIntake.BASE||getAngle()<IntakeConstants.basePosition) &&
            (currentPos!=PositionsIntake.FLOOR||getAngle()>IntakeConstants.floorPosition)
            )
                pivotMotor.getPIDController().setReference(pivotPos, ControlType.kPosition);
            else
                pivotMotor.set(0);
        }else{
            pivotMotor.set(pivotSpeed);
        }
        
        SmartDashboard.putNumber("Angle Intake", getAngle());
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
