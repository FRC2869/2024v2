// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Inputs;
import frc.robot.Constants.ShooterConstants.LightingSetting;

public class LightingSubsystem extends SubsystemBase {
  private static LightingSubsystem instance;
  private AddressableLED m_led;
  private AddressableLED m_led2;
  private AddressableLEDBuffer m_ledBuffer;
  private LightingSetting setting = LightingSetting.INTAKE;
  private int millis;
  private int player = 0;
  private boolean c1 = true;
  private boolean c2 = true;

  private int[][] level;

  public static LightingSubsystem getInstance() {
    if (instance == null) instance = new LightingSubsystem();
    return instance;
  }
  /** Creates a new LightingSubsystem. */
  public LightingSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(17);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setLights(LightingSetting lights) {
    setting = lights;
  }

  public void game() {
    setting = LightingSetting.GAME;
    level = new int[(int)(Math.random() * 3) + 5][2];
    for (int i = 0; i < level.length; i++) {
      level[i][0] = (int)(Math.random() * (m_ledBuffer.getLength() - 2)) + 1;
      level[i][1] = (int)(Math.random() * 4) + 1;
    }
  }

  private int counter = 0;
  private int speed = 10;
  @Override
  public void periodic() {
    counter += 1;
    if(counter%speed==0)
      millis += 1;
    if (Math.random() == 0) setting = LightingSetting.ORANGEWHITEGREENWITHBLUESPECKSITUATION;
    switch(setting){
      case SCORING:
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
      case INTAKE:
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
        break;
      case LOSE:
        player = 0;
        speed = 1;
        if (millis == 100) game();
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        break;
      case WIN:
        player = 0;
        speed = 1;
        if (millis == 500) game();
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }
        break;
      default:
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 150);
          }
  }
    m_led.setData(m_ledBuffer);
  }

  public void move(int move) {
    player += move;
  }
}
