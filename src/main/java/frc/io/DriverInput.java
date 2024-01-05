package frc.io;

import frc.util.LogitechF310Gamepad;

public class DriverInput {

  private static DriverInput instance;

  // Create empty driver and operator gamepads.
  private LogitechF310Gamepad driver;
  private LogitechF310Gamepad operator;

  // Creates boolean variables that stores if a certain step/mode was pressed
  private boolean autonIncreaseStepWasPressed = false;
  private boolean autonDecreaseStepWasPressed = false;

  private boolean autonIncreaseModeWasPressed = false;
  private boolean autonDecreaseModeWasPressed = false;

  private boolean autonIncreaseMode10WasPressed = false;
  private boolean autonDecreaseMode10WasPressed = false;

  /** Define driver and operator controllers. */
  private DriverInput() {
    this.driver = new LogitechF310Gamepad(0);
    this.operator = new LogitechF310Gamepad(1);
  }

  /**
   * Gets the current driver input instance; If there isn't one, create one.
   *
   * @return THe current driver input instance.
   */
  public static DriverInput getInstance() {
    if (instance == null) {
      instance = new DriverInput();
    }
    return instance;
  }

  /*****************************
   * DRIVER CONTROLS
   *****************************/

  // DRIVE
  public double getDriverRightX() {
    if (Math.abs(this.driver.getRightX()) < 0.05) {
      return 0;
    }
    return this.driver.getRightX();
  }

  public double getDriverLeftY() {
    if (Math.abs(this.driver.getLeftY()) < 0.05) {
      return 0;
    }
    return this.driver.getLeftY();
  }

  public double getDriverRightY() {
    if (Math.abs(this.driver.getRightY()) < 0.05) {
      return 0;
    }
    return this.driver.getRightY();
  }

  public double getDriverLeftX() {
    if (Math.abs(this.driver.getLeftX()) < 0.05) {
      return 0;
    }
    return this.driver.getLeftX();
  }

  public boolean getDriverManualOnButton() {
    return this.driver.getStartButton();
  }

  public boolean getDriverManualOffButton() {
    return this.driver.getBackButton();
  }

  public boolean getBalanceButton() {
    return this.driver.getLeftBumper();
  }

  public boolean getFaceGridButton() {
    return this.driver.getGreenButton();
  }

  public boolean getFaceStationButton() {
    return this.driver.getYellowButton();
  }

  public boolean getDriverAutoPilotButton() {
    return this.driver.getLeftTrigger() > 0.3;
  }

  public boolean getResetGyroButton() {
    return this.driver.getPOVUp();
  }

  public boolean getDriverLeftStationButton() {
    return this.driver.getPOVLeft();
  }

  public boolean getDriverRightStationButton() {
    return this.driver.getPOVRight();
  }

  // INTAKE

  public boolean getIntakeCubeButton() {
    return this.driver.getRightBumper();
  }

  public boolean getIntakeConeTippedButton() {
    return this.driver.getRedButton();
  }

  public boolean getIntakeConeButton() {
    return this.driver.getBlueButton();
  }

  public boolean getDriverScoreButton() {
    return this.driver.getRightTrigger() > 0.5;
  }

  /*****************************
   * OPERATOR CONTROLS
   *****************************/

  public boolean getOperatorManualOnButton() {
    return this.operator.getStartButton();
  }

  public boolean getOperatorCloseButton() {
    return this.operator.getBackButton();
  }

  public boolean getLeftBlinkerButton() {
    return this.operator.getPOVLeft();
  }

  public boolean getRightBlinkerButton() {
    return this.operator.getPOVRight();
  }

  /** ARM CONTROLS */

  /** Manual joystick controls */
  public double getArmManualProximal() {
    if (Math.abs(this.operator.getLeftY()) < 0.05) {
      return 0;
    } else {
      return -this.operator.getLeftY();
    }
  }

  public double getArmManualDistal() {
    if (Math.abs(this.operator.getRightY()) < 0.05) {
      return 0;
    } else {
      return this.operator.getRightY();
    }
  }

  public boolean getArmHoldingPositionButton() {
    return this.operator.getGreenButton();
  }

  /** Levels for scoring */
  public boolean getArmScoringLowButton() {
    return this.operator.getPOVDown();
  }

  public boolean getArmScoringMiddleButton() {
    return this.operator.getRedButton();
  }

  public boolean getArmScoringHighButton() {
    return this.operator.getYellowButton();
  }

  /**
   * Gets the button for setting the arm level to human loading station.
   *
   * @return The pressed state if the blue button on the operator controller.
   */
  public boolean getArmHumanLoadingLevelButton() {
    return this.operator.getBlueButton();
  }

  /** INTAKE/CLAW CONTROLS */

  /**
   * Resets the wrist encoder.
   *
   * @return The pressed state of the POV up button on the operator controller.
   */
  public boolean getWristEncoderResetButton() {
    return this.operator.getPOVUp();
  }

  /**
   * Gets the manual controls for opening and closing the wrist.
   *
   * @return The trigger inputs for controlling the wrist.
   */
  public double getWristManual() {
    return this.operator.getLeftTrigger() - this.operator.getRightTrigger();
  }

  public boolean getConeButton() {
    return this.operator.getRightBumper();
  }

  public boolean getCubeButton() {
    return this.operator.getLeftBumper();
  }

  // ********************************
  // AUTO SELECTION CONTROLS
  // ********************************

  public boolean getDriverAutoOverrideButtons() {
    return this.driver.getGreenButton();
  }

  public boolean getOperatorAutoOverrideButtons() {
    return false; // this.operator.getGreenButton();
  }

  public boolean getAutonSetDelayButton() {
    return false; // this.driver.getRightTrigger() > 0.2;
  }

  public double getAutonDelayStick() {
    return this.driver.getLeftY();
  }

  public boolean getAutonStepIncrease() {
    // only returns true on rising edge
    boolean result = this.driver.getRightBumper() && !this.autonIncreaseStepWasPressed;
    this.autonIncreaseStepWasPressed = this.driver.getRightBumper();
    return result;
  }

  public boolean getAutonStepDecrease() {
    // only returns true on rising edge
    boolean result = this.driver.getLeftBumper() && !this.autonDecreaseStepWasPressed;
    this.autonDecreaseStepWasPressed = this.driver.getLeftBumper();
    return result;
  }

  public boolean getAutonModeIncrease() {
    // only returns true on rising edge
    boolean result = this.driver.getRedButton() && !this.autonIncreaseModeWasPressed;
    this.autonIncreaseModeWasPressed = this.driver.getRedButton();
    return result;
  }

  public boolean getAutonModeDecrease() {
    // only returns true on rising edge
    boolean result = this.driver.getGreenButton() && !this.autonDecreaseModeWasPressed;
    this.autonDecreaseModeWasPressed = this.driver.getGreenButton();
    return result;
  }

  public boolean getAutonModeIncreaseBy10() {
    // only returns true on rising edge
    boolean result = this.driver.getYellowButton() && !this.autonIncreaseMode10WasPressed;
    this.autonIncreaseMode10WasPressed = this.driver.getYellowButton();
    return result;
  }

  public boolean getAutonModeDecreaseBy10() {
    // only returns true on rising edge
    boolean result = this.driver.getBlueButton() && !this.autonDecreaseMode10WasPressed;
    this.autonDecreaseMode10WasPressed = this.driver.getBlueButton();
    return result;
  }
}
