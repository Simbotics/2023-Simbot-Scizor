package frc.auton.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Drive;
import frc.subsystems.Intake;
import frc.util.SimPoint;

public class DriveSetPosition extends AutonCommand {

  // Positional
  private double x;
  private double y;
  private double angle;
  private boolean settingAutoStart;

  // Drive
  private IO io;
  private Drive drive;
  private Intake intake;

  // Declares the variables for the position of the robot using SimPoints
  public DriveSetPosition(SimPoint p, double angle) {
    this(true, p.getX(), p.getY(), angle);
  }

  public DriveSetPosition(Pose2d pose) {
    this(true, pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public DriveSetPosition(double x, double y, double angle) {
    this(true, x, y, angle);
  }

  // Declares the variables for the regular way of setting the position of the
  // robot
  public DriveSetPosition(boolean setAutoStart, double x, double y, double angle) {
    super(RobotComponent.DRIVE);

    // System.out.println("SET POS X: " + x);
    // System.out.println("SET POS Y: " + y);
    this.x = x;
    this.y = y;
    this.angle = angle;
    this.io = IO.getInstance();
    this.drive = Drive.getInstance();
    this.settingAutoStart = setAutoStart;

    this.intake = Intake.getInstance();
  }

  @Override
  public void firstCycle() {
    this.io.setDrivePose(this.x, this.y, this.angle);
    if (this.settingAutoStart) {
      this.drive.setAutoStartPose(
          new Pose2d(this.x, this.y, new Rotation2d(Math.toRadians(this.angle))));
    }

    // System.out.println("SETTING ANGLE" + this.angle);
    this.io.setDriveBrakeMode(true);
    this.drive.resetRateLimit();
    this.io.setClawEncoderPosition(40.0);
  }

  @Override
  // Sets the position of the robot on the field
  public boolean calculate() {
    this.io.setDrivePose(this.x, this.y, this.angle);
    if (this.settingAutoStart) {
      this.drive.setAutoStartPose(
          new Pose2d(this.x, this.y, new Rotation2d(Math.toRadians(this.angle))));
    }
    this.io.setDriveBrakeMode(true);
    this.io.setClawEncoderPosition(40.0);
    return true;
  }

  @Override
  // Unused method.
  public void override() {}
}
