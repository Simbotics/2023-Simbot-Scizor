package frc.util.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.IO;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriveConstants;
import frc.util.SimLimelight;
import frc.util.SimNavx;

// Wheels are a array numbered 0-3 from front to back, with even numbers on the left side when
//  facing forward.
public class SwerveDrive {

  private final double xyStdDevCoefficient = 1; // larger ==> ignore
  private final double thetaStdDevCoefficient = 0.01; // basically ignore this
  private final double poseEstimatorDistanceErrorCutoff = 2.0;
  private double visionAreaCutoff = RobotConstants.VISION_SCORING_AREA_CUTOFF;
  private SimNavx gyro;
  private int wheelCount = 4;
  private SwerveModule[] modules = new SwerveModule[wheelCount];
  private boolean isFieldOriented = true;
  private double autoStartAngle = 0;
  private boolean usingLimelights = false;

  // TODO: move these to IO
  private SimLimelight leftLimelight = new SimLimelight(RobotConstants.LIMELIGHT_LEFT);
  private SimLimelight rightLimelight = new SimLimelight(RobotConstants.LIMELIGHT_RIGHT);

  private final SimSwerveDrivePoseEstimator odometer =
      new SimSwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          new Rotation2d(0),
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0))
          },
          new Pose2d());

  public SwerveDrive(
      int[] steeringPorts,
      int[] drivingPorts,
      int[] encoderPorts,
      boolean[] drivingReversed,
      boolean[] turningReversed,
      double[] moduleOffsets,
      boolean[] absoluteReversed) {

    for (int i = 0; i < wheelCount; i++) {
      this.modules[i] =
          new SwerveModule(
              steeringPorts[i],
              drivingPorts[i],
              drivingReversed[i],
              turningReversed[i],
              encoderPorts[i],
              moduleOffsets[i],
              absoluteReversed[i]);
    }

    this.gyro = new SimNavx();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);
  }

  /**
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double xSpeed, double ySpeed, double turningSpeed) {
    ChassisSpeeds chassisSpeeds;

    xSpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

    turningSpeed *= DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    if (this.isFieldOriented) {
      // Relative to field
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, this.getRotation2d());
    } else {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    this.setModuleStates(moduleStates);
  }

  // TODO move to utils and name better
  public static SwerveModulePosition getSwerveModulePosition(SwerveModule module) {
    return new SwerveModulePosition(module.getDrivePosition(), module.getState().angle);
  }

  // TODO move to utils and name better
  public static SwerveModulePosition[] getSwerveModulePositions(SwerveModule[] modules) {
    var modulePositions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = getSwerveModulePosition(modules[i]);
    }
    return modulePositions;
  }

  public void updateOdometry() {
    this.gyro.update();
    odometer.update(getRotation2d(), getSwerveModulePositions(modules));

    if (RobotConstants.USING_LIMELIGHTS && usingLimelights) {
      this.addLimelightPoseToOdometryIfValid(leftLimelight, this.poseEstimatorDistanceErrorCutoff);
      this.addLimelightPoseToOdometryIfValid(rightLimelight, this.poseEstimatorDistanceErrorCutoff);
    }

    this.odometer.overrideAngle(Math.toRadians(this.getHeading()));

    // SmartDashboard.putNumberArray("Limelight results", results);
    // SmartDashboard.putNumber("LIMELIGHT X " , results[0]);

    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putNumber(
        "Odometer heading", odometer.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    for (int i = 0; i < modules.length; i++) {
      modules[i].updateDashboard();
    }
  }

  public double getModuleTLRaw() {
    return modules[0].getAbsoluteEncoderRad();
  }

  public double getModuleTRRaw() {
    return modules[1].getAbsoluteEncoderRad();
  }

  public double getModuleBLRaw() {
    return modules[2].getAbsoluteEncoderRad();
  }

  public double getModuleBRRaw() {
    return modules[3].getAbsoluteEncoderRad();
  }

  private SwerveModulePosition getModuleTLSwervePos() {
    return new SwerveModulePosition(0, getRotation2d());
  }

  public double getGyroSpeed() {
    return this.gyro.getGyroSpeed();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public void setUsingLimelights(boolean isUsingLimelight) {
    this.usingLimelights = isUsingLimelight;
  }

  private double distanceBetweenPoints(Pose2d a, Pose2d b) {
    return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
  }

  public void setVisionAreaCutoff(double cutoff) {
    this.visionAreaCutoff = cutoff;
  }

  public void setLimelightPipeline(int pipeline) {
    this.leftLimelight.setPipeline(pipeline);
    this.rightLimelight.setPipeline(pipeline);
  }

  public boolean getLeftLimelightValidTarget() {
    return this.leftLimelight.getTargetExists();
  }

  public boolean getRightLimelightValidTarget() {
    return this.rightLimelight.getTargetExists();
  }

  public double getLeftLimelightX() {
    if (this.leftLimelight.getTargetExists()) {
      return this.leftLimelight.getTargetX();
    } else {
      return 0;
    }
  }

  public double getRightLimelightX() {
    if (this.rightLimelight.getTargetExists()) {
      return this.rightLimelight.getTargetX();
    } else {
      return 0;
    }
  }

  public boolean isValidLimelightPose(SimLimelight limelight, double distance) {
    Pose2d limelightPose = limelight.getLimelightPose();
    if (!limelight.getTargetExists()) {
      return false;
    } else if (odometer.getEstimatedPosition().getX() == 0.0
        && odometer.getEstimatedPosition().getY() == 0.0) {
      return true;
    } else if (distanceBetweenPoints(odometer.getEstimatedPosition(), limelightPose) > distance
        && IO.getInstance().getIsAuto()) {
      return false;
    } else if (limelight.getTargetArea() < visionAreaCutoff) {
      return false;
    } else {
      return true;
    }
  }

  public void addLimelightPoseToOdometryIfValid(SimLimelight limelight, double distance) {
    limelight.putLimelightPoseValues();
    boolean isValid = this.isValidLimelightPose(limelight, distance);
    SmartDashboard.putBoolean(limelight.getName() + " updating pose", isValid);
    SmartDashboard.putNumber(limelight.getName() + " area", limelight.getTargetArea());
    if (isValid) {
      double distanceCoefficient = limelight.getTargetArea(); // todo: tune this
      double xy = this.xyStdDevCoefficient / distanceCoefficient;
      SmartDashboard.putNumber(limelight.getName() + " xy confidence", xy);
      double theta = thetaStdDevCoefficient;
      odometer.addVisionMeasurement(
          limelight.getLimelightPose(),
          limelight.getLimelightLatency(),
          VecBuilder.fill(xy, xy, theta));
    }
  }

  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    // TODO: Check what's supposed to be in modulePositions
    modules[0].resetEncoders();
    modules[1].resetEncoders();
    modules[2].resetEncoders();
    modules[3].resetEncoders();

    odometer.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
          new SwerveModulePosition(0, new Rotation2d(0)),
          new SwerveModulePosition(0, new Rotation2d(0)),
          new SwerveModulePosition(0, new Rotation2d(0)),
          new SwerveModulePosition(0, new Rotation2d(0)),
        },
        pose);
  }

  public void resetOdometry(double x, double y, double angle) {
    this.resetOdometry(new Pose2d(x, y, new Rotation2d(Math.toRadians(angle))));
  }

  public void resetOdometry() {
    this.resetOdometry(0, 0, 0);
  }

  public void setAutoStartAngle(double angle) {
    this.autoStartAngle = angle;
  }

  public double getX() {
    return this.odometer.getEstimatedPosition().getX();
  }

  public double getY() {
    return this.odometer.getEstimatedPosition().getY();
  }

  public void setFieldOriented(boolean fieldOriented) {
    this.isFieldOriented = fieldOriented;
  }

  public void setBrakeMode(boolean brake) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setBrakeMode(brake);
    }
  }

  public double getHeading() {
    // System.out.println("HEADING" + this.autoStartAngle);
    return gyro.getAngle() + this.autoStartAngle;
  }

  public double getAbsoluteHeading() {
    return gyro.getAbsoluteAngle() + this.autoStartAngle;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getRoll() {
    return gyro.getRoll();
  }
}
