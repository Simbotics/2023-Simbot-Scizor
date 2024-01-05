package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.auton.util.Reversible;
import frc.auton.util.ReversibleType;
import frc.io.IO;
import frc.subsystems.Drive;
import frc.subsystems.LEDStrip;
import frc.subsystems.LEDStrip.LEDColourState;

public class DriveBalance extends AutonCommand implements Reversible {

  private IO io;
  private LEDStrip ledStrip;
  private double eps;
  private double theta;
  private long timeout;

  public DriveBalance(double eps, double theta, long timeout) {
    super(RobotComponent.DRIVE, timeout);
    this.io = IO.getInstance();
    this.eps = eps;
    this.theta = theta;
    this.timeout = timeout;
    this.ledStrip = LEDStrip.getInstance();
  }

  @Override
  public void firstCycle() {
    this.io.setDriveBrakeMode(true);
  }

  @Override
  public boolean calculate() {
    this.ledStrip.setAllToState(LEDColourState.BALANCE_INDICATOR);
    return Drive.getInstance().balance(eps, theta);
  }

  @Override
  public void override() {
    this.ledStrip.setAllToState(LEDColourState.OFF);
    io.drive(0, 0, 0);
  }

  @Override
  public DriveBalance reverse(ReversibleType type) {
    switch (type) {
      case ALLIANCE:
        return new DriveBalance(this.eps, 180 - this.theta, this.timeout);
      case BUMP:
        return new DriveBalance(this.eps, -this.theta, this.timeout);
      case SLOW:
        return this;
      default:
        return this;
    }
  }
}
