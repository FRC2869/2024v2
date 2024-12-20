package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.MotorConfiguration;

/**
 * Controls the speed of the intake and 
 * measures if the note has been fully intaked in autonomous.
 * @author Encore "Raghavan"
 */
public class IntakeSpinSubsystem extends SubsystemBase{
    private static IntakeSpinSubsystem instance;
    public static IntakeSpinSubsystem getInstance(){
        if (instance == null) instance = new IntakeSpinSubsystem();
        return instance;
    }

    private TalonFX spinMotor;
    // private CANSparkFlex pivotMotor;
    private double spinSpeed;
    // private boolean isPosControl = false;
    // private double pivotPos;
    // private double pivotSpeed;
    // private PositionsIntake currentPos = PositionsIntake.BASE;
    private Timer time;

    public IntakeSpinSubsystem() {
        spinMotor = new TalonFX(15);
        configureMotors();
        // System.out.println("ayya");
        time = new Timer();
        time.reset();
        time.start();
    }

    /**
     * Configures the motor I guess...
     */
    private void configureMotors() {
        MotorConfiguration.configureMotor(spinMotor, IntakeConstants.spinMotorConfig);
    }

    /**
     * Sets speed to 100%, spinning notes into the robot
     */
    public void spinIn() {
        spinSpeed = .8;
        time.reset();
    }
/**
 * public static boolean smoke(){
 *  return true;
 * }
 */
    /**
     * Sets speed to 100%, spinning notes out of the robot
     */
    public void spinOut(){
        spinSpeed = -1;
        time.reset();
    }

    /**
     * Sets speed to 0%
     */
    public void spinStop(){
        spinSpeed = 0;
    }

    /**
     * 
     * @return a boolean stating true if the current spikes more than .25 seconds after the intake changes direction (signifying a note is inside the intake)
     */
    public boolean isIntake() {
        return (time.get()>0.25)&&(spinMotor.getSupplyCurrent().getValue() > 24.5);
        // return (spinMotor.getVelocity().getValue() < 3);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("intake current",spinMotor.getSupplyCurrent().getValue());
        SmartDashboard.putBoolean("Is intaked?", isIntake());
        SmartDashboard.putNumber("Dream Speed", spinSpeed);
        SmartDashboard.putNumber("Reality check (sad)", spinMotor.get());
        spinMotor.set(spinSpeed);
    }
}
