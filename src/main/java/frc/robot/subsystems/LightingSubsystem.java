// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Inputs;
import frc.robot.Constants.ShooterConstants.LightingSetting;

/**
 * Controls the LED's.
 * @author Johnny (Shi Huang Di) Chen
 */
public class LightingSubsystem extends SubsystemBase {
  private static LightingSubsystem instance;
  private AddressableLED m_led;
  // private AddressableLED m_led2;
  private AddressableLEDBuffer m_ledBuffer;
  private LightingSetting setting = LightingSetting.AUTO;
  private int millis;

  /**
   * The white LED in my game
   * @author the white led in my game
   */
  private int player = 0;
  private boolean c1 = true;
  private boolean c2 = true;
  private boolean aBool = true;
  private LightingSetting[] arrayOfc = {LightingSetting.Pattern1, LightingSetting.Pattern2, LightingSetting.Pattern3, LightingSetting.Pattern4};
  private static int numCalls;
  private int timer = 0;

  private int[][] level;

  /**
   * Gets an instance
   * @return the instance!!!
   */
  public static LightingSubsystem getInstance() {
    if (instance == null) instance = new LightingSubsystem();
    return instance;
  }
  /** Creates a new LightingSubsystem. */
  public LightingSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    m_ledBuffer = new AddressableLEDBuffer(17);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /**
   * Sets lights
   * WARNING: DO NOT CALL 24 TIMES.  YOU CAN CALL MORE THAN 24 TIMES BUT NOT EXACTLY 24!
   * @param lights sets the mode of the lights
   */
  public void setLights(LightingSetting lights) {
    setting = lights;
    numCalls++;
    if (numCalls == 24) setting = LightingSetting.GAMEOVER;
  }

  /**
   * idk what this does...
   */
  public void game() {
    timer = -3;
    player = 0;
    millis = 0;
    setting = LightingSetting.GAME;
    level = new int[(int)(Math.random() * 3) + 5][2];
    for (int i = 0; i < level.length; i++) {
      level[i][0] = (int)(Math.random() * (m_ledBuffer.getLength() - 2)) + 1;
      level[i][1] = (int)(Math.random() * 4) + 1;
    }
    random = false;
  }

  private int counter = 0;
  private int speed = 10;
  private boolean random;
  @Override
  public void periodic() {
    counter += 1;
    if(counter%speed == 0) {
      millis += 1;
    }
    if (Math.random() == 0) setting = LightingSetting.ORANGEWHITEGREENWITHBLUESPECKSITUATION;
    if (random) {
      if (counter > 800) 
      {
        randomPat();
      }
      if (Inputs.getChange()  && aBool) {
        game();
        aBool = false;
      }
      else if (!Inputs.getChange()) aBool = true;
    }
    switch(setting){
      case TELEOP:
        speed = 10; 
        millis %= 2;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if ((millis == 1 && (i % 4 == 0||i%4==1)) || (millis == 0 && (i % 4 == 2||i%4==3)))
            m_ledBuffer.setRGB(i, 0, 0, 255);
          if ((millis == 0 && (i % 4 == 0||i%4==1)) || (millis == 1 && (i % 4 == 2||i%4==3)))
            m_ledBuffer.setRGB(i, 255, 255, 255);
        }
        break;
      case CANSHOOT:
        speed = 1;
        millis %= 2;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 255, 255);
        }
        break;
      case AUTO:
        speed = 5;
        millis %= 6;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if ((i + millis) % 6 == 0)
            m_ledBuffer.setRGB(i, 255, 0, 0);
          if ((i + millis) % 6 == 1)
            m_ledBuffer.setRGB(i, 255, 255, 0);
          if ((i + millis) % 6 == 2)
            m_ledBuffer.setRGB(i, 0, 255, 0);
          if ((i + millis) % 6 == 3)
            m_ledBuffer.setRGB(i, 0, 255, 255);
          if ((i + millis) % 6 == 4)
            m_ledBuffer.setRGB(i, 0, 0, 255);
          if ((i + millis) % 6 == 5)
            m_ledBuffer.setRGB(i, 255, 0, 255);
        }
        break;
      case ORANGEWHITEGREENWITHBLUESPECKSITUATION:
        speed = 1;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if (i < m_ledBuffer.getLength()/3)
              m_ledBuffer.setRGB(i, 255, 150, 0);
            else if (i < 2 * m_ledBuffer.getLength()/3) {
              m_ledBuffer.setRGB(i, 255, 255, 255);
              if (i == (int)(m_ledBuffer.getLength()/2))
                m_ledBuffer.setRGB(i, 0, 0, 255);
            }
            else {
              m_ledBuffer.setRGB(i, 0, 255, 0);
            }
          }
        break;
      case INTAKING:
        speed = 5;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if ((millis == 1 && (i % 4 == 0||i%4==1)) || (millis == 0 && (i % 4 == 2||i%4==3)))
            m_ledBuffer.setRGB(i, 255, 0, 0);
          if ((millis == 0 && (i % 4 == 0||i%4==1)) || (millis == 1 && (i % 4 == 2||i%4==3)))
            m_ledBuffer.setRGB(i, 200, 100, 100);
        }
      case DISABLED:
        speed = 1;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if (i < m_ledBuffer.getLength()/3)
              m_ledBuffer.setRGB(i, 255, 150, 0);
            else if (i < 2 * m_ledBuffer.getLength()/3) {
              m_ledBuffer.setRGB(i, 255, 255, 255);
              if (i == (int)(m_ledBuffer.getLength()/2))
                m_ledBuffer.setRGB(i, 0, 0, 255);
            }
            else {
              m_ledBuffer.setRGB(i, 0, 255, 0);
            }
          }
        break;
      case GAME:
        if (Inputs.getChange() && aBool) {
          randomPat();
          aBool = false;
        }
        else if (!Inputs.getChange()) aBool = true;
        speed = 1;
        //System.out.println(Inputs.getRight());
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 0, 0, 255);
        }
        for (int i = 0; i < level.length; i++) {
          if (millis % (level[i][1] * 100) >= level[i][1]*50) {
            m_ledBuffer.setRGB(level[i][0], 255, 0, 0);
            if (player == level[i][0]) {
              setting = LightingSetting.LOSE;
              millis = 0;
            }
          }
          else m_ledBuffer.setRGB(level[i][0], 0, 255, 0);
        }
        m_ledBuffer.setRGB(player, 255, 255, 255);
        if (Inputs.getLeft() && player != 0 && c1) {
          player--;
          c1 = false;
        }
        else if (!Inputs.getLeft()) c1 = true;
        if (Inputs.getRight() && player != m_ledBuffer.getLength()  && c2){
           player++;
           c2 = false;
        }
        else if (!Inputs.getRight()) c2 = true;
        if (player == m_ledBuffer.getLength()) {
          millis = 0;
          setting = LightingSetting.WIN;
        }
        for (int i = 0; i < timer - 1; i++) {
          m_ledBuffer.setRGB(i, 255, 0, 0);
          if (player == i) {
            millis = 0;
            setting = LightingSetting.LOSE;
          }
        }
        if (millis % 50 == 0 && timer != m_ledBuffer.getLength()) {
          timer++;
        }
        break;
      case LOSE:
        player = 0;
        speed = 1;
        if (millis == 100) game();
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        break;
      case Pattern1:
        speed = 1;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if (millis % (510) < 255)
            m_ledBuffer.setRGB(i, 0, 0, millis % 255);
          else
            m_ledBuffer.setRGB(i, 0, 0, 255 - (millis % 255));
        }
        break;
      case Pattern2:
        speed = 1;
        if (millis % 100 == 0) {
          for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if(Math.random() < .3)
              m_ledBuffer.setRGB(i, 0, 0, 255);
            else
              m_ledBuffer.setRGB(i, 0, 0, 0);
          }
        }
        break;
      case Pattern3:
        speed = 1;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if(i == millis % m_ledBuffer.getLength() || i == (millis + 1) % m_ledBuffer.getLength() || i == (millis + 2) % m_ledBuffer.getLength())
            m_ledBuffer.setRGB(i, 0, 0, 255);
          else
            m_ledBuffer.setRGB(i, 75, 0, 255);
        }
        break;
      case Pattern4:
        speed = 5;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if((i + millis) % 4 == 0)
            m_ledBuffer.setRGB(i, 61, 161, 104);
          else if((i + millis) % 4 == 1)
            m_ledBuffer.setRGB(i, 0, 255, 255);
          else if((i + millis) % 4 == 2)
            m_ledBuffer.setRGB(i, 0, 0, 255);
          else
            m_ledBuffer.setRGB(i, 101, 0, 184);
        }
        break;
      case WIN:
        player = 0;
        speed = 1;
        if (millis == 200) game();
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }
        break;
      case GAMEOVER:
        speed = 1;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if (millis % (550) < 270)
            m_ledBuffer.setRGB(i, millis, 0, 0);
          else
            m_ledBuffer.setRGB(i, 550 - millis, 0, 0);
        }
        break;
      default:
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 0, 0, 150);
        }
  }
    m_led.setData(m_ledBuffer);
  }

  /**
   * This is a function (it runs when called)
   */
  public void randomPat() {
    counter = 0;
    random = true;
    setting = arrayOfc[(int)(Math.random() * arrayOfc.length)];
  }
}
