package frc.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.io.IO;

public class LEDStrip {

  private static LEDStrip instance;

  LEDColourState[] states =
      new LEDColourState[] {
        LEDColourState.SOLID_RED,
        LEDColourState.SOLID_RED,
        LEDColourState.SOLID_RED,
        LEDColourState.SOLID_RED
      };
  AddressableLED leds;
  AddressableLEDBuffer buffer;
  IO io;
  int cycle = 0;

  // All possible states of the LED's.
  public enum LEDColourState {
    OFF,
    RAINBOW,
    BREATHING_RED,
    HALF_BLUE_HALF_RED,
    SNAKE_RAINBOW_ALL,
    SNAKE_RAINBOW_CYCLE,
    PING_PONG,
    PANIC,
    ALL_WHITE,
    SOLID_RED,
    SOLID_GREEN,
    SOLID_BLUE,
    VISION_AIMED,
    VISION_TOO_CLOSE,
    VISION_TARGET,
    VISION_NO_TARGET,
    HUMAN_LOADING_STATION_CUBE,
    HUMAN_LOADING_STATION_CONE,
    INTAKED_CONE,
    INTAKED_CUBE,
    BALANCE_INDICATOR
  }

  private LEDStrip() {
    this.leds = new AddressableLED(0);
    this.buffer = new AddressableLEDBuffer(64);
    this.io = IO.getInstance();
    leds.setLength(64);

    this.update();
    leds.start();
  }

  public static LEDStrip getInstance() {
    if (instance == null) {
      instance = new LEDStrip();
    }
    return instance;
  }

  public LEDColourState getLEDState(Segment segment) {
    return this.states[segment.index];
  }

  public void setFront(LEDColourState state) {
    this.states[Segment.FrontLeft.index] = state;
    this.states[Segment.FrontRight.index] = state;
  }

  public void setBack(LEDColourState state) {
    this.states[Segment.BackLeft.index] = state;
    this.states[Segment.BackRight.index] = state;
  }

  public void setLeft(LEDColourState state) {
    this.states[Segment.FrontLeft.index] = state;
    this.states[Segment.BackLeft.index] = state;
  }

  public void setRight(LEDColourState state) {
    this.states[Segment.FrontRight.index] = state;
    this.states[Segment.BackRight.index] = state;
  }

  public void setSegment(LEDColourState state, Segment segment) {
    states[segment.index] = state;
  }

  public void setSegments(
      LEDColourState frontLeft,
      LEDColourState backLeft,
      LEDColourState backRight,
      LEDColourState frontRight) {
    states = new LEDColourState[] {frontLeft, backLeft, backRight, frontRight};
  }

  public void clearAllButt(LEDColourState state, Segment segment) {
    states =
        new LEDColourState[] {
          LEDColourState.OFF, LEDColourState.OFF, LEDColourState.OFF, LEDColourState.OFF
        };
    states[segment.index] = state;
  }

  public void setDefault() {
    this.setAllToState(LEDColourState.SOLID_RED);
  }

  public void setAllToState(LEDColourState state) {
    states = new LEDColourState[] {state, state, state, state};
  }

  public enum Segment {
    FrontLeft(0),
    BackLeft(1),
    BackRight(2),
    FrontRight(3);

    public final int index;

    private Segment(int index) {
      this.index = index;
    }
  }

  /**
   * Changes the state of the LED
   *
   * <p>0 - front right side 1 - back right side 2 - back left side 3 - front left side
   *
   * @param states of the LED strip segments as an array
   */
  public void setLEDState(LEDColourState[] states) {
    this.states = states;
  }

  public void update() {
    // Why did I do it this way?
    for (int seg = 0; seg < states.length; seg++) {
      int minSegWindow = seg * 16;
      int maxSegWindow = minSegWindow + 16;

      switch (this.states[seg]) {
        case HUMAN_LOADING_STATION_CONE -> {
          if (DriverStation.getMatchTime() <= 35.0) {
            double rainbowSpeed = 5.0;
            double hue = System.currentTimeMillis() / rainbowSpeed;
            for (int i = minSegWindow; i < maxSegWindow; i++) {
              buffer.setHSV(i, (int) ((hue + i * 5) % 180), 255, 255);
            }
          } else {
            if (System.currentTimeMillis() % 666 < 333) {
              for (int i = minSegWindow; i < maxSegWindow; i++) {
                buffer.setRGB(i, 255, 128, 0);
              }
            } else {
              for (int i = minSegWindow; i < maxSegWindow; i++) {
                buffer.setRGB(i, 0, 0, 0);
              }
            }
          }
        }
        case HUMAN_LOADING_STATION_CUBE -> {
          if (System.currentTimeMillis() % 666 < 333) {
            for (int i = minSegWindow; i < maxSegWindow; i++) {
              buffer.setRGB(i, 255, 0, 255);
            }
          } else {
            for (int i = minSegWindow; i < maxSegWindow; i++) {
              buffer.setRGB(i, 0, 0, 0);
            }
          }
        }
        case INTAKED_CONE -> {
          for (int i = minSegWindow; i < maxSegWindow; i++) {
            buffer.setRGB(i, 255, 128, 0);
          }
        }
        case INTAKED_CUBE -> {
          for (int i = minSegWindow; i < maxSegWindow; i++) {
            buffer.setRGB(i, 255, 0, 255);
          }
        }
        case BREATHING_RED -> {
          Color color = new Color((Math.sin(Math.PI * this.cycle / 80.0) + 1.0) / 2.0, 0, 0);

          for (int i = minSegWindow; i < maxSegWindow; i++) {
            buffer.setLED(i, color);
          }
        }

        case RAINBOW -> {
          double rainbowSpeed = 15.0;
          double hue = System.currentTimeMillis() / rainbowSpeed;
          for (int i = minSegWindow; i < maxSegWindow; i++) {
            buffer.setHSV(i, (int) ((hue + i * 5) % 180), 255, 255);
          }
        }

        case OFF -> {
          for (int i = minSegWindow; i < maxSegWindow; i++) {
            buffer.setRGB(i, 0, 0, 0);
          }
        }

        case HALF_BLUE_HALF_RED -> {
          // First half
          for (int i = minSegWindow; i < maxSegWindow / 2; i++) {
            buffer.setRGB(i, 0, 255, 0);
          }

          // Second half
          for (int i = maxSegWindow / 2; i < maxSegWindow; i++) {
            buffer.setRGB(i, 0, 0, 255);
          }
        }

        case SNAKE_RAINBOW_ALL -> {
          int snakeLength = 15;
          double snakeSpeed = 100.0;
          int snakeBegin = (int) ((System.currentTimeMillis() / snakeSpeed) % maxSegWindow);
          int snakeEnd = (snakeBegin + snakeLength) % maxSegWindow;

          for (int i = minSegWindow; i < maxSegWindow; i++) {

            boolean isWithin;
            if (snakeEnd < snakeBegin) {
              isWithin = i >= snakeBegin || i <= snakeEnd;
            } else {
              isWithin = i >= snakeBegin && i <= snakeEnd;
            }
            if (isWithin) {
              int hue = (i - snakeBegin) * 179 / snakeLength;
              if (hue < 0) {
                hue = (i + maxSegWindow - snakeBegin) * 179 / snakeLength;
              }
              buffer.setHSV(i, hue, 255, 255);
            } else {
              buffer.setRGB(i, 0, 0, 0);
            }
          }
        }

        case SNAKE_RAINBOW_CYCLE -> {
          double snakeSpeed = 250.0;
          double rainbowSpeed = 15.0;
          int snakeLength = 15;
          int snakeBegin = (int) ((System.currentTimeMillis() / snakeSpeed) % maxSegWindow);
          int snakeEnd = (snakeBegin + snakeLength) % maxSegWindow;

          for (int i = minSegWindow; i < maxSegWindow; i++) {

            boolean isWithin;
            if (snakeEnd < snakeBegin) {
              isWithin = i >= snakeBegin || i <= snakeEnd;
            } else {
              isWithin = i >= snakeBegin && i <= snakeEnd;
            }
            if (isWithin) {
              double hue = System.currentTimeMillis() / rainbowSpeed;
              buffer.setHSV(i, (int) (hue % 180), 255, 255);
            } else {
              buffer.setRGB(i, 0, 0, 0);
            }
          }
        }

        case PING_PONG -> {
          int snakeBegin = (int) ((System.currentTimeMillis() / 250.0) % (maxSegWindow * 2));

          if (snakeBegin > maxSegWindow) {
            snakeBegin = maxSegWindow * 2 - snakeBegin;
          }

          for (int i = minSegWindow; i < maxSegWindow; i++) {

            if (i == snakeBegin) {
              buffer.setHSV(i, 0, 255, 255);
            } else {
              buffer.setRGB(i, 0, 0, 0);
            }
          }
        }

        case BALANCE_INDICATOR -> {
          for (int i = minSegWindow; i < maxSegWindow; i++) {
            int red = (int) Math.abs(io.getGyroPitch()) * 8;
            if (red > 255) red = 255;
            int green = 255 - red;

            buffer.setRGB(i, red, green, 0);
          }
        }

        case PANIC -> {
          if (cycle % 5 == 0) {
            for (int i = minSegWindow; i < maxSegWindow; i++) {
              buffer.setHSV(i, (int) (Math.random() * 180), 255, 255);
            }
          }
        }
        case SOLID_RED -> {
          for (int i = minSegWindow; i < maxSegWindow; i++) {
            buffer.setRGB(i, 255, 0, 0);
          }
        }
        case SOLID_GREEN -> {
          for (int i = minSegWindow; i < maxSegWindow; i++) {
            buffer.setRGB(i, 0, 255, 0);
          }
        }
        case SOLID_BLUE -> {
          for (int i = minSegWindow; i < maxSegWindow; i++) {
            buffer.setRGB(i, 0, 0, 255);
          }
        }
        default -> throw new IllegalArgumentException(
            "Unhandled LED state (Someone should implement it!): " + this.states[seg]);
      }
    }
    this.cycle += 1;
    leds.setData(buffer);
  }

  public void setBlinkingCone() {
    this.setAllToState(LEDColourState.HUMAN_LOADING_STATION_CONE);
  }

  public void setBlinkingCube() {
    this.setAllToState(LEDColourState.HUMAN_LOADING_STATION_CUBE);
  }

  public void setBlinkingLeft() {
    if (this.getLEDState(Segment.FrontRight) == LEDColourState.HUMAN_LOADING_STATION_CONE
        || this.getLEDState(Segment.FrontRight) == LEDColourState.HUMAN_LOADING_STATION_CUBE) {
      this.states[Segment.FrontLeft.index] = this.states[Segment.FrontRight.index];
      this.states[Segment.BackLeft.index] = this.states[Segment.BackRight.index];
      this.setRight(LEDColourState.OFF);
    }
  }

  public void setBlinkingRight() {
    if (this.getLEDState(Segment.FrontLeft) == LEDColourState.HUMAN_LOADING_STATION_CONE
        || this.getLEDState(Segment.FrontLeft) == LEDColourState.HUMAN_LOADING_STATION_CUBE) {
      this.states[Segment.FrontRight.index] = this.states[Segment.FrontLeft.index];
      this.states[Segment.BackRight.index] = this.states[Segment.BackLeft.index];
      this.setLeft(LEDColourState.OFF);
    }
  }
}
