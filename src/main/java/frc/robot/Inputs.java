package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A class containing static functions for each of the inputs
 * Demo (only active when Operator A is pressed):
 * Left X/Y - Translation
 * Right X - Rotation
 * A Button - Shoot
 * 
 * Operator:
 * Left X/Y - Translation
 * Right X - Rotation
 * A Button - Enable Demo Controller
 * B Button - Intake Down
 * X Button - Intake Up
 * Y Button - Reset Gyro
 * 
 * Operator Board:
 * 1: Intake note command (fancy)
 * 2: Transfer for intake
 * 3: Shoots (fancy)
 * 4: Amp transfer
 * 5: Brings pivot to floor
 * 6: Base position of intake
 * 7: Far position of intake
 * 8: Intake spin out
 * 9: Intake spin stop
 * 10: Intake spin in
 * 11: Shoots (not fancy)
 * 12: Far position of shooter
 * 13: Base position of shooter
 * 14: Amp position of shooter
 * 15: Shoots far
 * 
 * @author ChatGPT
 */
public class Inputs {
    /**
     * Start intake - XboxController X
     * Start outtake - Xbox
     */
    private static final CommandXboxController demo = new CommandXboxController(Constants.OperatorConstants.DriverController.port); 
    private static final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.DriverController2.port); 
    // private static final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.OperatorController.port); 
    private static final CommandGenericHID operatorBoard = new CommandGenericHID(Constants.OperatorConstants.OperatorButtonBoard.port); 

    public static Trigger getResetGyro(){
        return operator.y();
    }
    
    // public static Trigger getClimberDown(){
    //     return operatorBoard.button(1);
    // }

    // public static Trigger getClimberUp(){
    //     return operatorBoard.button(2);
    // }

    // public static Trigger getShooterShoot(){
    //     return operator.pov(90);
    // }

    // public static Trigger getShooterAmpScore(){
    //     return operator.a();
    // }

    // public static Trigger getShooterAmpLoad(){
    //     return operator.pov(0);
    // }

    // public static Trigger getShooterStop(){
    //     return operator.leftBumper();
    // }

    // public static boolean getSlowMode(){
    //     return demo.getHID().getRightBumper()||demo.getHID().getLeftBumper();
    // }

    public static double getTranslationX() {
        var speed = -operator.getLeftY();
        if (getDemoEnabled())
            speed += -demo.getLeftY()*.25;
        return speed;
    }

    private static boolean getDemoEnabled() {
        return operator.getHID().getAButton();
    }

    public static double getRotation() {
        var speed = -operator.getRightX();
        if (getDemoEnabled())
            speed += -demo.getRightX()*.25;
        return speed;
    }

    // private static LinkedList<Double> speedList = new LinkedList<>();
     
    public static double getTranslationY() {
        var speed = -operator.getLeftX();
        if (getDemoEnabled())
            speed += -demo.getLeftX()*.25;
        return speed;
        
    }

    // public static double getIntakePivotSpeed() {

    //     var speed =  operator.getHID().getLeftX()*.5;
    //     if(Math.abs(speed)<.1){
    //         speed = 0;
    //     }
    //     return speed;
    // }

    // public static Trigger getIntakeSpinIn() {
    //     return operator.x();
    // }

    // public static Trigger getIntakeSpinOut() {
    //     return operator.y();
    // }

    // public static Trigger getIntakeSpinStop() {
    //     return operator.b();
    // }

    // public static double getPivotOverride(){
    //     return operator.getHID().getRightY()*.5;
    // }

    // public static Trigger getPivotBase(){
    //     return operator.pov(180);
    // }

    // public static Trigger getPivotAmp(){
    //     return operator.pov(270);
    // }

    
    public static Trigger getAutoIntakeDown(){
        return operatorBoard.button(1);
    }

    public static Trigger getAutoIntakeDown2(){
        return operator.b();
    }

    public static Trigger getAutoIntakeUp(){
        return operatorBoard.button(2);
    }

    public static Trigger getAutoIntakeUp2(){
        return operator.x();
    }

    public static Trigger getAutoShootStop(){
        return operatorBoard.button(3);
    }

    public static Trigger getAutoShootStop2(){
        return demo.a();
    }

    public static Trigger getAutoShootStop3(){
        return operator.pov(0);
    }

    public static Trigger getAmpTransfer(){
        return operatorBoard.button(4);
    }

    public static Trigger getIntakeFloorPos() {
        return operatorBoard.button(5);
    }

    public static Trigger getIntakeBasePos() {
        return operatorBoard.button(6);
    }

    public static Trigger getIntakeFar(){
        return operatorBoard.button(7);
    }

    public static Trigger getIntakeSpinOut2() {
        return operatorBoard.button(8);
    }

    public static Trigger getIntakeSpinStop2() {
        return operatorBoard.button(9);
    }

    public static Trigger getIntakeSpinIn2() {
        return operatorBoard.button(10);
    }

    public static Trigger getShooterShoot2(){
        return operatorBoard.button(11);
    }

    public static Trigger getPivotFar(){
        return operatorBoard.button(12);
    }

    public static Trigger getPivotBase2(){
        return operatorBoard.button(13);
    }

    public static Trigger getPivotAmp2(){
        return operatorBoard.button(14);
    }

    public static Trigger getShooterFarShoot(){
        return operatorBoard.button(15);
    }
    
    // public static Trigger getAimAtSpeaker(){
    //     // return operatorBoard.button();
    //     return operatorBoard.button(16);
    // }
    // public static Trigger getMoveToAmp(){
    //     // return operatorBoard.button();
    //     return operatorBoard.button(17);
    // }

    public static Trigger getShooterAdjustUp(){
        // return operatorBoard.button();
        return operatorBoard.button(16);
    }
    public static Trigger getShooterAdjustDown(){
        // return operatorBoard.button();
        return operatorBoard.button(17);
    }
    public static Trigger getIntakeAdjustUp(){
        // return operatorBoard.button();
        return operatorBoard.button(18);
    }
    public static Trigger getIntakeAdjustDown(){
        // return operatorBoard.button();
        return operatorBoard.button(19);
    }

    public static Trigger getShooterStop2(){
        return operatorBoard.button(20);
    }
    public static Trigger getAmpAutoOuttake(){
        return operatorBoard.button(21);
    }
    public static boolean getLoadAuto(){
        return operatorBoard.getHID().getRawButton(22);
    }
    // for auto-align calibration
    public static Trigger getAngles() {
        return null;
    }

    public static boolean getLeft() {
        return demo.getHID().getPOV()==270;
    }
    public static boolean getRight() {
        return demo.getHID().getPOV()==90;
    }
    public static boolean getChange() {
        return false;
        // return driver1.getHID().getRawButton(11);
    }

    public static Trigger getRobotCentric() {
        return demo.rightTrigger();
    }

}

