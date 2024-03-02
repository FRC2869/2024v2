package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Inputs {
    private static final CommandXboxController driver = new CommandXboxController(Constants.OperatorConstants.DriverController.port); 
    private static final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.OperatorController.port); 
    private static final CommandGenericHID operatorBoard = new CommandGenericHID(Constants.OperatorConstants.OperatorButtonBoard.port); 

    public static Trigger getShooterShoot(){
        return operatorBoard.button(1);
    }

    public static Trigger getShooterAmpScore(){
        return operatorBoard.button(2);
    }

    public static Trigger getShooterAmpLoad(){
        return operatorBoard.button(3);
    }

    public static Trigger getShooterStop(){
        return operatorBoard.button(4);
    }

    public static double getTranslationX() {
        return driver.getLeftX();
    }

    public static double getRotation() {
        return driver.getRightX();
    }

    public static double getTranslationY() {
        return driver.getLeftY(); 
    }

    public static double getIntakePivotSpeed() {
        return operator.getLeftX();
    }

    public static Trigger getIntakeFloorPos() {
        return operatorBoard.button(1);
    }

    public static Trigger getIntakeBasePos() {
        return operatorBoard.button(2);
    }

    public static Trigger getIntakeSpinIn() {
        return operatorBoard.button(3);
    }

    public static Trigger getIntakeSpinOut() {
        return operatorBoard.button(4);
    }

    public static Trigger getIntakeSpinStop() {
        return operatorBoard.button(5);
    }
    
}
