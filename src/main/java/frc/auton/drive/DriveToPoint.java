package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.auton.util.Reversible;
import frc.auton.util.ReversibleType;
import frc.io.IO;
import frc.subsystems.Drive;

public class DriveToPoint extends AutonCommand implements Reversible {

  private double x;
  private double y;
  private double theta; // The desired angle
  private double minVelocity;
  private double maxVelocity;
  private double eps; // The acceptable range
  private double turnSpeed;
  private IO io;
  private boolean autoOriginRelative;
  private long timeout;
  private boolean usingLimelights;

  private Drive drive;

  public DriveToPoint(double x, double y, double theta, long timeout) {
    this(x, y, theta, 0.0, 0.03, timeout);
  }

  public DriveToPoint(
      double x, double y, double theta, double minVelocity, double eps, long timeout) {
    this(x, y, theta, minVelocity, 1.0, eps, timeout);
  }

  public DriveToPoint(
      double x,
      double y,
      double theta,
      double minVelocity,
      double maxVelocity,
      double eps,
      long timeout) {
    this(x, y, theta, minVelocity, maxVelocity, 0.7, eps, timeout);
  }

  public DriveToPoint(
      double x,
      double y,
      double theta,
      double minVelocity,
      double maxVelocity,
      double turnSpeed,
      double eps,
      long timeout) {
    this(x, y, theta, minVelocity, maxVelocity, turnSpeed, eps, timeout, true, false);
  }

  public DriveToPoint(
      double x,
      double y,
      double theta,
      double minVelocity,
      double maxVelocity,
      double turnSpeed,
      double eps,
      long timeout,
      boolean autoOriginRelative,
      boolean usingLimelights) {
    super(RobotComponent.DRIVE, timeout);
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
    this.eps = eps;
    this.turnSpeed = turnSpeed;
    this.autoOriginRelative = autoOriginRelative;
    this.timeout = timeout;
    this.usingLimelights = usingLimelights;

    this.io = IO.getInstance();
    this.drive = Drive.getInstance();
    this.io.setDriveBrakeMode(true);
  }

  @Override
  public void firstCycle() {
    this.io.setOdometryUsingLimelight(this.usingLimelights);
  }

  @Override
  public boolean calculate() {
    boolean isDone =
        this.drive.driveToPoint(
            x, y, theta, minVelocity, maxVelocity, turnSpeed, eps, autoOriginRelative);
    return isDone;
  }

  @Override
  public void override() {
    this.io.drive(0, 0, 0);
    this.drive.resetRateLimit();
  }

  @Override
  public DriveToPoint reverse(ReversibleType type) {
    switch (type) {
      case ALLIANCE:
        return new DriveToPoint(
            -x,
            y,
            180 - theta,
            minVelocity,
            maxVelocity,
            turnSpeed,
            eps,
            timeout,
            autoOriginRelative,
            usingLimelights);
      case BUMP:
        return new DriveToPoint(
            x,
            -y,
            -theta,
            minVelocity,
            maxVelocity,
            turnSpeed,
            eps,
            timeout,
            autoOriginRelative,
            usingLimelights);
      case SLOW:
        return new DriveToPoint(
            x,
            y,
            theta,
            minVelocity * 0.7,
            maxVelocity * 0.7,
            turnSpeed,
            eps,
            timeout,
            autoOriginRelative,
            usingLimelights);
      default:
        return this;
    }
  }
}
