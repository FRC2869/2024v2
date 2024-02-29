package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Inputs {
    private static final CommandXboxController driver = new CommandXboxController(Constants.OperatorConstants.DriverController.port); 
    private static final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.OperatorController.port); 
    private static final CommandGenericHID operatorBoard = new CommandGenericHID(Constants.OperatorConstants.OperatorButtonBoard.port); 


    public static double getPivotOverride(){
        return operator.getRightY();
    }

    public static Trigger getPivotBase(){
        return operatorBoard.button(1);
    }

    public static Trigger getPivotAmp(){
        return operatorBoard.button(2);
    }

    public static Trigger getPivotFar(){
        return operatorBoard.button(3);
    }
    
}
