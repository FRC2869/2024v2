package frc.robot;

import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Inputs {
    private static final CommandXboxController driver = new CommandXboxController(Constants.OperatorConstants.DriverController.port); 
    private static final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.OperatorController.port); 
    private static final CommandGenericHID operatorBoard = new CommandGenericHID(Constants.OperatorConstants.OperatorButtonBoard.port); 

    public static Trigger getResetGyro(){
        return driver.y();
    }

    public static Trigger getShooterShoot(){
        return operator.pov(90);
    }

    public static Trigger getShooterAmpScore(){
        return operator.a();
    }

    public static Trigger getShooterAmpLoad(){
        return operator.pov(0);
    }

    public static Trigger getShooterStop(){
        return operator.leftBumper();
    }

    public static double getTranslationX() {
        return -driver.getLeftX();
    }

    public static double getRotation() {
        return -driver.getRightX();
    }

    private static LinkedList<Double> speedList = new LinkedList<>();
     
    public static double getTranslationY() {
        var speed = driver.getLeftY(); 
        // speedList.removeFirst();
		// speedList.add(speed);

        // double avg = 0;
		// for (int i = 0; i < speedList.size(); i++) {
		// 	avg += speedList.get(i);
		// }
		// avg /= speedList.size();
		return speed; 
        
    }

    public static double getIntakePivotSpeed() {
        return operator.getLeftX();
    }

    public static Trigger getIntakeFloorPos() {
        return operatorBoard.button(5);
    }

    public static Trigger getIntakeBasePos() {
        return operatorBoard.button(6);
    }

    public static Trigger getIntakeSpinIn() {
        return operator.x();
    }

    public static Trigger getIntakeSpinOut() {
        return operator.y();
    }

    public static Trigger getIntakeSpinStop() {
        return operator.b();
    }

    public static double getPivotOverride(){
        return operator.getRightY();
    }

    public static Trigger getPivotBase(){
        return operatorBoard.button(10);
    }

    public static Trigger getPivotAmp(){
        return operatorBoard.button(11);
    }

    public static Trigger getPivotFar(){
        return operatorBoard.button(12);
    }
    
}
