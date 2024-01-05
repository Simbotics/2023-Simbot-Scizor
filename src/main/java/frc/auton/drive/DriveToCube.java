package frc.auton.drive;

import frc.auton.AutonBase;
import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.auton.util.Reversible;
import frc.auton.util.ReversibleType;
import frc.io.IO;
import frc.subsystems.Drive;

public class DriveToCube extends AutonCommand implements Reversible {

  private IO io;

  private double theta;
  private double velocity;
  private double turnSpeed;
  private long timeout;
  private boolean right;

  public DriveToCube(double velocity, double theta, double turnSpeed, boolean right, long timeout) {
    super(RobotComponent.DRIVE, timeout);
    this.io = IO.getInstance();

    this.theta = theta;
    this.timeout = timeout;
    this.turnSpeed = turnSpeed;
    this.velocity = velocity;
    this.right = right;
  }

  @Override
  public void firstCycle() {
    this.io.setDriveBrakeMode(true);
  }

  @Override
  public boolean calculate() {
    return Drive.getInstance().driveToCube(this.theta, this.velocity, this.turnSpeed, this.right);
  }

  @Override
  public void override() {
    io.drive(0, 0, 0);
    io.setLimelightPipeline(0);
  }

  @Override
  public AutonBase reverse(ReversibleType type) {
    switch (type) {
      case ALLIANCE:
        return new DriveToCube(velocity, 180.0 - theta, turnSpeed, !this.right, this.timeout);
      case BUMP:
        return new DriveToCube(velocity, -theta, turnSpeed, !this.right, this.timeout);
      case SLOW:
        return new DriveToCube(velocity, theta, turnSpeed, this.right, this.timeout + 500);
      default:
        return this;
    }
  }
}
