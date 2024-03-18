package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.MotorConfiguration;

/**
 * Controls the speed of the intake and 
 * measures if the note has been fully intaked in autonomous.
 * @author Encore "Raghavan"
 */
public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem instance;
    public static IntakeSubsystem getInstance(){
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    private TalonFX spinMotor;
    // private CANSparkFlex pivotMotor;
    private double spinSpeed;
    // private boolean isPosControl = false;
    // private double pivotPos;
    // private double pivotSpeed;
    // private PositionsIntake currentPos = PositionsIntake.BASE;

    public IntakeSubsystem() {
        spinMotor = new TalonFX(15);
        configureMotors();
        // System.out.println("ayya");
    }

    /**
     * Configures the motor I guess...
     */
    private void configureMotors() {
        MotorConfiguration.configureMotor(spinMotor, IntakeConstants.spinMotorConfig);
    }

    /**
     * Sets speed to 50%, spinning notes into the robot
     */
    public void spinIn() {
        spinSpeed = .5;
    }

    /**
     * Sets speed to 100%, spinning notes out of the robot
     */
    public void spinOut(){
        spinSpeed = -1;
    }

    /**
     * Sets speed to 0%
     */
    public void spinStop(){
        spinSpeed = 0;
    }

    /**
     * 
     * @return a boolean stating true if the velocity decreases (signifying a note is inside the intake)
     */
    public boolean isIntake() {
        //return (spinMotor.getSupplyCurrent().getValue() > 15);
        return (spinMotor.getVelocity().getValue() < 3);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("intake current",spinMotor.getSupplyCurrent().getValue());
        SmartDashboard.putBoolean("Is intaked?", isIntake());
        spinMotor.set(spinSpeed);
    }
}
