package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;

public class DriveOutput extends AutonCommand {

  private IO io;
  private double output;

  /**
   * Declares needed variables.
   *
   * @param output maxOutput
   * @param timeoutLength rampRate
   */
  public DriveOutput(double output, long timeoutLength) {
    super(RobotComponent.DRIVE, timeoutLength);
    this.output = output;
    this.io = IO.getInstance();
  }

  @Override
  public void firstCycle() {
    this.io.setFieldOriented(false);
    this.io.setDriveBrakeMode(true);
  }

  @Override
  /** Sets the motor output for turning. */
  public boolean calculate() {
    this.io.drive(output, 0, 0);
    return false;
  }

  @Override
  /** Stops the robot when activated. */
  public void override() {
    this.io.drive(0, 0, 0);
  }
}
