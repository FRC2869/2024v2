package frc.robot;

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
        var speed = -driver.getLeftY();
        if(driver.getHID().getRightBumper()){
            return speed*.25;
        }
        return speed;
    }

    public static double getRotation() {
        var speed =  -driver.getRightX();
        if(driver.getHID().getRightBumper()){
            return speed*.25;
        }
        return speed;
    }

    // private static LinkedList<Double> speedList = new LinkedList<>();
     
    public static double getTranslationY() {
        var speed = -driver.getLeftX(); 
        if(driver.getHID().getRightBumper()){
            return speed*.25;
        }
        return speed;
        // speedList.removeFirst();
		// speedList.add(speed);

        // double avg = 0;
		// for (int i = 0; i < speedList.size(); i++) {
		// 	avg += speedList.get(i);
		// }
		// avg /= speedList.size();
		// return speed; 
        
    }

    public static double getIntakePivotSpeed() {

        var speed =  operator.getHID().getLeftX()*.5;
        if(Math.abs(speed)<.1){
            speed = 0;
        }
        return speed;
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
        return operator.getHID().getRightY()*.5;
    }

    public static Trigger getPivotBase(){
        return operator.pov(180);
    }

    public static Trigger getPivotAmp(){
        return operator.pov(270);
    }

    public static Trigger getPivotFar(){
        return operatorBoard.button(12);
    }
    public static Trigger getIntakeFar(){
        return operatorBoard.button(7);
    }
    
    public static Trigger getAutoIntakeDown(){
        return operatorBoard.button(1);
    }
    public static Trigger getAutoIntakeUp(){
        return operatorBoard.button(2);
    }

    public static Trigger getAutoShoot(){
        return operatorBoard.button(3);
    }

    // for auto-align calibration
    public static Trigger getAngles() {
        return null;
    }
}

