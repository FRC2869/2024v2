package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A class containing static functions for each of the inputs
 * Driver1 (drives):
 * 2: reset gyro
 * 3: Climber Down
 * 4: Climber Up
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
 * 22: climb up
 * 23: climb down
 * 25: climb down
 * 
 * @author ChatGPT
 */
public class Inputs {
    /**
     * Start intake - XboxController X
     * Start outtake - Xbox
     */
    private static final XboxController controllerLol = new XboxController(Constants.OperatorConstants.DriverController.port);
    private static final CommandXboxController driver1 = new CommandXboxController(Constants.OperatorConstants.DriverController.port); 
    // private static final CommandXboxController driver2 = new CommandXboxController(Constants.OperatorConstants.DriverController2.port); 
    // private static final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.OperatorController.port); 
    private static final CommandGenericHID operatorBoard = new CommandGenericHID(Constants.OperatorConstants.OperatorButtonBoard.port); 

    public static Trigger getResetGyro(){
        return driver1.button(2);
    }
    
    public static Trigger getClimberDown(){
        return operatorBoard.button(1);
    }

    public static Trigger getClimberUp(){
        return operatorBoard.button(2);
    }

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

    public static boolean getSlowMode(){
        return driver1.getHID().getRightBumper()||driver1.getHID().getLeftBumper();
    }
    // DID YOU JUST UPDATE CTRE SWERVE? Remember to invert the drive motors in TunerConstants
    public static double getTranslationX() {
        var speed = -driver1.getLeftY();
        if(getSlowMode()){
            return speed*.25;
        }
        return speed;
    }

    public static double getRotation() {
        var speed =  -driver1.getRightX();
        if(getSlowMode()){
            return speed*.25;
        }
        return speed;
    }

    // private static LinkedList<Double> speedList = new LinkedList<>();
     
    public static double getTranslationY() {
        var speed = -driver1.getLeftX(); 
        if(getSlowMode()){
            return speed*.25;
        }
        return speed;
        // speedList.removeFirst();
		// speedList.add(speed);

        // doubl..................................e avg = 0;
		// for (int i = 0; i < speedList.size(); i++) {
		// 	avg += speedList.get(i);
		// }
		// avg /= speedList.size();
		// return speed; 
        
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
    public static Trigger getAutoIntakeUp(){
        return operatorBoard.button(2);
    }

    public static Trigger getAutoShootStop(){
        return operatorBoard.button(3);
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

    public static Trigger getAutoAimShooter() {
        return operatorBoard.button(25);
    }

    public static Trigger getAutoAimShooter2() {
        return operatorBoard.button(26);
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
        return driver1.getHID().getPOV()==270;
    }
    public static boolean getRight() {
        return driver1.getHID().getPOV()==90;
    }
    public static boolean getChange() {
        return false;
        // return driver1.getHID().getRawButton(11);
    }

    public static Trigger getPivotReset(){
        return operatorBoard.button(31);
    }

    public static Trigger getToggleFaceSpeaker() {
        return driver1.povDown();
    }

    public static Trigger getRobotCentric() {
        return driver1.rightTrigger();
    }

    public static Trigger getClimberMovingUp(){
        return operatorBoard.button(27);
    }

    public static Trigger getClimberMovingDown(){
        return operatorBoard.button(23);
    }

    public static Trigger getSourceIntake() {
        return operatorBoard.button(24);
    }


    public static Trigger getShooterIntake() {
        return operatorBoard.button(26);
    }

    public static Trigger getPivotMoveUp() {
        return operatorBoard.button(32);
    }

    public static Trigger getPivotMoveDown() {
        return operatorBoard.button(31);
    }

    public static Trigger getRUMBLE() {
        return operatorBoard.button(29);
    }

    public static void RUMBLERUMBLE(double rumble) {
        try {
            controllerLol.setRumble(RumbleType.kLeftRumble, rumble);
        } catch (Exception e) {}
    }

    // public static Trigger getAutoIntakeUp2() {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'getAutoIntakeUp2'");
    // }

    // public static Trigger getAutoIntakeDown2() {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'getAutoIntakeDown2'");
    // }

    // public static Trigger getAutoShootStop3() {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'getAutoShootStop3'");
    // }

    // public static Trigger getAutoShootStop2() {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'getAutoShootStop2'");
    // }
    
    public static Trigger getAutoAlignShooter() {
        // TODO Auto-generated method stub
        return driver1.povUp();
    }

    public static Trigger getPivotClimbPosition() {
        return operatorBoard.button(28);
    }

    public static Trigger getIntakeFromShooter() {
        return operatorBoard.button(30);
    }

    // public static Trigger getTurnToSpeaker() {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'getTurnToSpeaker'");
    // }

    // public static Trigger goToAmp() {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'goToAmp'");
    // }

}

