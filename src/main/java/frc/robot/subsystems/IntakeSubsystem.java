package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfiguration;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PositionsIntake;

public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem instance;
    public static IntakeSubsystem getInstance(){
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    private TalonFX spinMotor;
    private CANSparkFlex pivotMotor;
    private double spinSpeed;
    private boolean isPosControl = false;
    private double pivotPos;
    private double pivotSpeed;
    private PositionsIntake currentPos = PositionsIntake.BASE;

    public IntakeSubsystem() {
        spinMotor = new TalonFX(15, null);
        configureMotors();
    }

    private void configureMotors() {
        MotorConfiguration.configureMotor(spinMotor, IntakeConstants.spinMotorConfig);
    }

    public void spinIn() {
        spinSpeed = .5;
    }

    public void spinOut(){
        spinSpeed = -.5;
    }

    public void spinStop(){
        spinSpeed = 0;
    }

    @Override
    public void periodic(){
        spinMotor.set(spinSpeed);
    }
}
