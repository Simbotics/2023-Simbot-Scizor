/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.IO;

/** Add your docs here. */
public class SimLimelight {

  private NetworkTable netWorkTable;
  private String name;
  private IO io;

  public SimLimelight(String name) {
    this.name = name;

    this.netWorkTable = NetworkTableInstance.getDefault().getTable(name);

    this.netWorkTable.getEntry("camera").setNumber(0);
    this.netWorkTable.getEntry("pipeline").setNumber(0);
  }

  public String getName() {
    return this.name;
  }

  public void setLEDMode(int ledMode) { // 0 is pipeline default, 1 is off, 2 is blink, 3 is on
    this.netWorkTable.getEntry("ledMode").setNumber(ledMode);
  }

  public void setStream(int mode) {
    this.netWorkTable.getEntry("stream").setNumber(mode);
  }

  public void setPipeline(int pipeline) {
    this.netWorkTable.getEntry("pipeline").setNumber(pipeline);
  }

  public double getTargetX() {
    return this.netWorkTable.getEntry("tx").getDouble(0); // because portrait
  }

  public double getTargetY() {
    return this.netWorkTable.getEntry("ty").getDouble(0); // because portrait
  }

  public boolean getTargetExists() {
    return this.netWorkTable.getEntry("tv").getInteger(0) == 1;
  }

  public double getTargetArea() {
    return this.netWorkTable.getEntry("ta").getDouble(0);
  }

  public double getTargetRotation() {
    return this.netWorkTable.getEntry("ts").getDouble(0);
  }

  public double[] getTargetPointsXY() {
    return this.netWorkTable.getEntry("tcornxy").getDoubleArray(new double[10]);
  }

  public Pose2d getLimelightPose() {
    double[] results = this.netWorkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    Pose3d cameraPose =
        new Pose3d(
            results[0],
            -results[1],
            results[2],
            new Rotation3d(results[3], results[4], Math.toRadians(IO.getInstance().getHeading())));
    Pose2d pose = cameraPose.toPose2d();
    return pose;
  }

  public void putLimelightPoseValues() {
    Pose2d pose = this.getLimelightPose();
    SmartDashboard.putNumber(this.name + " x", pose.getX());
    SmartDashboard.putNumber(this.name + " y", pose.getY());
    SmartDashboard.putNumber(this.name + " angleDegrees", pose.getRotation().getDegrees());
  }

  public double getLimelightLatency() {
    double[] results = this.netWorkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    return Timer.getFPGATimestamp() - (results[6] / 1000.0);
  }
}
