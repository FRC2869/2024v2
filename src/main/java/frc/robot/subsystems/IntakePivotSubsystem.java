package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfiguration;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PositionsIntake;

public class IntakePivotSubsystem extends SubsystemBase{
    private static IntakePivotSubsystem instance;
    public static IntakePivotSubsystem getInstance(){
        if (instance == null) instance = new IntakePivotSubsystem();
        return instance;
    }

    private TalonFX spinMotor;
    private CANSparkFlex pivotMotor;
    private double spinSpeed;
    private boolean isPosControl = false;
    private double pivotPos;
    private double pivotSpeed;
    private PositionsIntake currentPos = PositionsIntake.BASE;
    private boolean isCoast = false;

    public IntakePivotSubsystem() {
        pivotMotor = new CANSparkFlex(16, null);
        configureMotors();
    }

    private void configureMotors() {
        MotorConfiguration.configureMotor(pivotMotor, IntakeConstants.pivotMotorConfig);
    }

    public void setPivotPos(double pos){
        this.pivotPos = MathUtil.clamp(pos, IntakeConstants.kMinAngle, IntakeConstants.kMaxAngle);
    }

    public void setPivotSpeed(double speed){
        this.pivotSpeed = speed;
    }

    public void resetPivot() {
        pivotMotor.getEncoder().setPosition(IntakeConstants.basePosition);
    }

    public double getAngle(){
        return pivotMotor.getEncoder().getPosition();
    }

    public double getVelocity(){
        return pivotMotor.getEncoder().getVelocity();
    }

    public boolean isAtPosition(){
		if(currentPos == PositionsIntake.BASE){
            return getAngle()>IntakeConstants.basePosition;
        }else{
            return getAngle()<IntakeConstants.floorPosition;
        }
			
	}

    public void adjustUp() {
		pivotPos += 2;
	}

	public void adjustDown() {
		pivotPos -= 2;
	}

    public void setCurrentPosition(PositionsIntake pos) {
		currentPos = pos;
	}

    @Override
    public void periodic(){
        if (isPosControl) {
            if(getAngle()>0&&getAngle()<110)
                pivotMotor.getPIDController().setReference(pivotPos, ControlType.kPosition);
            else
                pivotMotor.set(0);
        }else{
            pivotMotor.set(pivotSpeed);
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
