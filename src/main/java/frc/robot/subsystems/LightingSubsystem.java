// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.LightingSetting;
import frc.robot.Inputs;
/**
 * Controls the LED's.
 * 
 * @author Johnny (Shi Huang Di) Chen
 */
public class LightingSubsystem extends SubsystemBase {
    private static LightingSubsystem instance;
    public CANdle candle;
    private CANdleConfiguration config;
    private int timer;
    private int player;
    private int[][] level;
    private LightingSetting[] arrayOfc = {LightingSetting.Pattern1, LightingSetting.Pattern2, LightingSetting.Pattern3, LightingSetting.Pattern4};    private int numCalls;
    /**
     * The white LED in my game
     * 
     * @author the white led in my game
     */
    /**
     * Gets an instance
     * 
     * @return the instance!!!
     */
    public static LightingSubsystem getInstance() {
        if (instance == null)
            instance = new LightingSubsystem();
        return instance;
    }
    /**

//    * Sets lights

//    * WARNING: DO NOT CALL 24 TIMES.  YOU CAN CALL MORE THAN 24 TIMES BUT NOT EXACTLY 24!

//    * @param lights sets the mode of the lights

//    */

  public void setLights(LightingSetting lights) {

    setting = lights;

    numCalls++;

    if (numCalls == 24) setting = LightingSetting.GAMEOVER;

  }
    /** Creates a new LightingSubsystem. */
    public LightingSubsystem() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        candle = new CANdle(9, "CAN FD bus");
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.GRB;
        config.disableWhenLOS = false;
        config.brightnessScalar = 1;
        config.vBatOutputMode = VBatOutputMode.Off;
        // candle.getAllConfigs(config);
        candle.configAllSettings(config);
        for (int i=0;i<getLength();i++)
            setRGB(i, 0, 0, 255);
    }
    public int getLength(){
        return 58;
    }
/**

//    * This is a function (it runs when called)

//    */

  public void randomPat() {

    counter = 0;

    random = true;

    setting = arrayOfc[(int)(Math.random() * arrayOfc.length)];

  }

    /**

//    * idk what this does...

//    */

  public void game() {

    timer = -3;

    player = 8;

    millis = 0;

    setting = LightingSetting.GAME;

    level = new int[(int)(Math.random() * 15) + 5][2];

    for (int i = 0; i < level.length; i++) {

      level[i][0] = (int)(Math.random() * (getLength() - 2)) + 1;

      level[i][1] = (int)(Math.random() * 4) + 1;

    }

    random = false;

  }
    
    private int counter = 0;
    private int speed = 10;
    private boolean random;
    private int millis;
    private LightingSetting setting = LightingSetting.DISABLED;
    private boolean aBool;
    private boolean c1;
    private boolean c2;
    @Override
    public void periodic() {
        // counter += 1;
        // if (counter % speed == 0) {
        //     millis += 1;
        // }
        // if (Math.random() == 0)
        //     setting = LightingSetting.ORANGEWHITEGREENWITHBLUESPECKSITUATION;
        // if (random) {
        //     if (counter > 800)
        //     {
        //         randomPat();
        //     }
        //     if (Inputs.getChange() && aBool) {
        //         game();
        //         aBool = false;
        //     }
        //     else if (!Inputs.getChange())
        //         aBool = true;
        // }
        // switch (setting) {
        //     case TELEOP:
        //         speed = 10;
        //         millis %= 2;
        //         for (int i = 0; i < getLength(); i++) {
        //             if ((millis == 1 && (i % 4 == 0 || i % 4 == 1)) || (millis == 0 && (i % 4 == 2 || i % 4 == 3)))
        //                 setRGB(i, 0, 0, 255);
        //             if ((millis == 0 && (i % 4 == 0 || i % 4 == 1)) || (millis == 1 && (i % 4 == 2 || i % 4 == 3)))
        //                 setRGB(i, 255, 255, 255);
        //         }
        //         break;
        //     case CANSHOOT:
        //         speed = 1;
        //         millis %= 2;
        //         for (int i = 0; i < getLength(); i++) {
        //             setRGB(i, 255, 255, 255);
        //         }
        //         break;
        //     case AUTO:
        //         speed = 5;
        //         millis %= 6;
        //         for (int i = 0; i < getLength(); i++) {
        //             if ((i + millis) % 6 == 0)
        //                 setRGB(i, 255, 0, 0);
        //             if ((i + millis) % 6 == 1)
        //                 setRGB(i, 255, 255, 0);
        //             if ((i + millis) % 6 == 2)
        //                 setRGB(i, 0, 255, 0);
        //             if ((i + millis) % 6 == 3)
        //                 setRGB(i, 0, 255, 255);
        //             if ((i + millis) % 6 == 4)
        //                 setRGB(i, 0, 0, 255);
        //             if ((i + millis) % 6 == 5)
        //                 setRGB(i, 255, 0, 255);
        //         }
        //         break;
        //     case ORANGEWHITEGREENWITHBLUESPECKSITUATION:
        //         speed = 1;
        //         for (int i = 0; i < getLength(); i++) {
        //             if (i < getLength() / 3)
        //                 setRGB(i, 255, 150, 0);
        //             else if (i < 2 * getLength() / 3) {
        //                 setRGB(i, 255, 255, 255);
        //                 if (i == (int) (getLength() / 2))
        //                     setRGB(i, 0, 0, 255);
        //             }
        //             else {
        //                 setRGB(i, 0, 255, 0);
        //             }
        //         }
        //         break;
        //     case INTAKING:
        //         speed = 5;
        //         for (int i = 0; i < getLength(); i++) {
        //             if ((millis == 1 && (i % 4 == 0 || i % 4 == 1)) || (millis == 0 && (i % 4 == 2 || i % 4 == 3)))
        //                 setRGB(i, 255, 0, 0);
        //             if ((millis == 0 && (i % 4 == 0 || i % 4 == 1)) || (millis == 1 && (i % 4 == 2 || i % 4 == 3)))
        //                 setRGB(i, 200, 100, 100);
        //         }
        //     case DISABLED:
        //         speed = 1;
        //         for(int i=0; i<getLength();i++){
        //             setRGB(i, 100, 100, 100);
        //         }
        //         // for (int i = 0; i < getLength(); i++) {
        //         //     if (i < getLength() / 3)
        //         //         setRGB(i, 255, 150, 0);
        //         //     else if (i < 2 * getLength() / 3) {
        //         //         setRGB(i, 255, 255, 255);
        //         //         if (i == (int) (getLength() / 2))
        //         //             setRGB(i, 0, 0, 255);
        //         //     }
        //         //     else {
        //         //         setRGB(i, 0, 255, 0);
        //         //     }
        //         // }
        //         break;
        //     case GAME:
        //         int[][] lev = new int[getLength()][3];
        //         if (Inputs.getChange() && aBool) {
        //             randomPat();
        //             aBool = false;
        //         }
        //         else if (!Inputs.getChange())
        //             aBool = true;
        //         speed = 1;
        //         // System.out.println(Inputs.getRight());
        //         for (int i = 0; i < getLength(); i++) {
        //             lev[i][2] = 255;
        //             // setRGB(i, 0, 0, 255);
        //         }
        //         for (int i = 0; i < level.length; i++) {
        //             if (millis % (level[i][1] * 100) >= level[i][1] * 50) {
        //                 lev[level[i][0]][0] = 255;
        //                 lev[level[i][0]][2] = 0;
        //                 // setRGB(level[i][0], 255, 0, 0);
        //                 if (player == level[i][0]) {
        //                     setting = LightingSetting.LOSE;
        //                     millis = 0;
        //                 }
        //             }
        //             else {
        //                 lev[level[i][0]][1] = 255;
        //                 lev[level[i][0]][2] = 0;
        //                 // setRGB(level[i][0], 0, 255, 0);
        //             }
        //         }
        //         // setRGB(player, 255, 255, 255);
        //         lev[player][0] = 255;
        //         lev[player][1] = 255;
        //         lev[player][2] = 255;
        //         if (Inputs.getLeft() && player != 0 && c1) {
        //             player--;
        //             c1 = false;
        //         }
        //         else if (!Inputs.getLeft())
        //             c1 = true;
        //         if (Inputs.getRight() && player != getLength() && c2) {
        //             player++;
        //             c2 = false;
        //         }
        //         else if (!Inputs.getRight())
        //             c2 = true;
        //         if (player == getLength()) {
        //             millis = 0;
        //             setting = LightingSetting.WIN;
        //         }
        //         for (int i = 0; i < timer - 1; i++) {
        //             // setRGB(i, 255, 0, 0);
        //             lev[i][0] = 255;
        //             lev[i][2] = 0;
        //             if (player == i) {
        //                 millis = 0;
        //                 setting = LightingSetting.LOSE;
        //             }
        //         }
        //         if (millis % 50 == 0 && timer != getLength()) {
        //             timer++;
        //         }
        //         for (int i = 0; i < lev.length; i++) {
        //             setRGB(i, lev[i][0], lev[i][1], lev[i][2]);
        //         }
        //         break;
        //     case LOSE:
        //         player = 8;
        //         speed = 1;
        //         if (millis == 100)
        //             game();
        //         for (int i = 0; i < getLength(); i++) {
        //             setRGB(i, 255, 0, 0);
        //         }
        //         break;
        //     case Pattern1:
        //         speed = 1;
        //         for (int i = 0; i < getLength(); i++) {
        //             if (millis % (510) < 255)
        //                 setRGB(i, 0, 0, millis % 255);
        //             else
        //                 setRGB(i, 0, 0, 255 - (millis % 255));
        //         }
        //         break;
        //     case Pattern2:
        //         speed = 1;
        //         if (millis % 100 == 0) {
        //             for (int i = 0; i < getLength(); i++) {
        //                 if (Math.random() < .3)
        //                     setRGB(i, 0, 0, 255);
        //                 else
        //                     setRGB(i, 0, 0, 0);
        //             }
        //         }
        //         break;
        //     case Pattern3:
        //         speed = 1;
        //         for (int i = 0; i < getLength(); i++) {
        //             if (i == millis % getLength() || i == (millis + 1) % getLength()
        //                     || i == (millis + 2) % getLength())
        //                 setRGB(i, 0, 0, 255);
        //             else
        //                 setRGB(i, 75, 0, 255);
        //         }
        //         break;
        //     case Pattern4:
        //         speed = 5;
        //         for (int i = 0; i < getLength(); i++) {
        //             if ((i + millis) % 4 == 0)
        //                 setRGB(i, 61, 161, 104);
        //             else if ((i + millis) % 4 == 1)
        //                 setRGB(i, 0, 255, 255);
        //             else if ((i + millis) % 4 == 2)
        //                 setRGB(i, 0, 0, 255);
        //             else
        //                 setRGB(i, 101, 0, 184);
        //         }
        //         break;
        //     case WIN:
        //         player = 8;
        //         speed = 1;
        //         if (millis == 200)
        //             game();
        //         for (int i = 0; i < getLength(); i++) {
        //             setRGB(i, 0, 255, 0);
        //         }
        //         break;
        //     case GAMEOVER:
        //         speed = 1;
        //         for (int i = 0; i < getLength(); i++) {
        //             if (millis % (550) < 270)
        //                 setRGB(i, millis, 0, 0);
        //             else
        //                 setRGB(i, 550 - millis, 0, 0);
        //         }
        //         break;
        //     default:
        //         for (int i = 0; i < getLength(); i++) {
        //             setRGB(i, 250, 0, 0);
        //         }
        // }
        
    }
    private void setRGB(int i, int r, int g, int b) {
        candle.setLEDs(r,g,b,0, i, 1);
    }
}