package frc.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class LogitechF310Gamepad {

  // Define the gamepad used.
  private Joystick joystick;

  /**
   * Define all gamepad variables.
   *
   * @param portNumber Port number gamepad is plugged into.
   */
  public LogitechF310Gamepad(int portNumber) {
    this.joystick = new Joystick(portNumber);
  }

  /**
   * Gets the left joystick X axis.
   *
   * @return The left joystick X axis.
   */
  public double getLeftX() {
    return this.joystick.getRawAxis(0);
  }

  /**
   * Gets the left joystick Y axis.
   *
   * @return The left joystick Y axis.
   */
  public double getLeftY() {
    return -this.joystick.getRawAxis(1);
  }

  /**
   * Gets the pressed state of the left trigger.
   *
   * @return The pressed state of the left trigger as a boolean.
   */
  public double getLeftTrigger() {
    return this.joystick.getRawAxis(2);
  }

  /**
   * Gets the pressed state of the right trigger.
   *
   * @return The pressed state of the right trigger as a boolean.
   */
  public double getRightTrigger() {
    return this.joystick.getRawAxis(3);
  }

  /**
   * Gets the right joystick X axis.
   *
   * @return The right joystick X axis.
   */
  public double getRightX() {
    return this.joystick.getRawAxis(4);
  }

  /**
   * Gets the right joystick Y axis.
   *
   * @return The right joystick Y axis.
   */
  public double getRightY() {
    return -this.joystick.getRawAxis(5);
  }

  /** Button methods. */

  /**
   * Gets the buttons pressed state as a boolean.
   *
   * @param btnNum Button number identifier.
   * @return The buttons state as a boolean.
   */
  public boolean getButton(int btnNum) {
    return this.joystick.getRawButton(btnNum);
  }

  /**
   * Gets the green buttons pressed state.
   *
   * @return The pressed state of the greeen button as a boolean.
   */
  public boolean getGreenButton() {
    return this.joystick.getRawButton(1);
  }

  /**
   * Gets the blue buttons pressed state.
   *
   * @return The pressed state of the blue button as a boolean.
   */
  public boolean getBlueButton() {
    return this.joystick.getRawButton(3);
  }

  /**
   * Gets the red buttons pressed state.
   *
   * @return The pressed state of the red button as a boolean.
   */
  public boolean getRedButton() {
    return this.joystick.getRawButton(2);
  }

  /**
   * Gets the yellow buttons pressed state.
   *
   * @return The pressed state of the yellow button as a boolean.
   */
  public boolean getYellowButton() {
    return this.joystick.getRawButton(4);
  }

  public boolean getBackButton() {
    return this.joystick.getRawButton(7);
  }

  public boolean getStartButton() {
    return this.joystick.getRawButton(8);
  }

  /**
   * Gets the pressed state of the top left back button.
   *
   * @return The value of the pressed state as a boolean.
   */
  public boolean getLeftBumper() {
    return this.joystick.getRawButton(5);
  }

  /**
   * Gets the pressed state of the top right back button.
   *
   * @return The value of the pressed state as a boolean.
   */
  public boolean getRightBumper() {
    return this.joystick.getRawButton(6);
  }

  /**
   * Gets the pressed state of the left joystick button.
   *
   * @return The state of the left joystick button as a boolean.
   */
  public boolean getLeftStickClick() {
    return this.joystick.getRawButton(9);
  }

  /**
   * Gets the pressed state of the right joystick button.
   *
   * @return The state of the of the right joystick button as a boolean.
   */
  public boolean getRightStickClick() {
    return this.joystick.getRawButton(10);
  }

  /** POV methods. */
  public int getPOVVal() {
    return this.joystick.getPOV(0);
  }

  public boolean getPOVDown() {
    return (this.joystick.getPOV(0) == 180);
  }

  public boolean getPOVRight() {
    return (this.joystick.getPOV(0) == 90);
  }

  public boolean getPOVUp() {
    return (this.joystick.getPOV(0) == 0);
  }

  public boolean getPOVLeft() {
    return (this.joystick.getPOV(0) == 270);
  }

  /**
   * Sets the amount of rumble for supported controllers.
   *
   * @param rumble Amount of rumble; Either 0 or 1.
   */
  public void setRumble(double rumble) {
    this.joystick.setRumble(RumbleType.kLeftRumble, rumble);
    this.joystick.setRumble(RumbleType.kRightRumble, rumble);
  }
}
