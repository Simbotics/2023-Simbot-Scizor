package frc.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.subsystems.LEDStrip.LEDColourState;
import frc.util.SimPID;
import frc.util.SimPoint;
import frc.util.Vect;
import java.util.List;

public class Drive extends SubsystemBase {

  private static Drive instance;
  private DriveState currentDriveState;

  private TrackingState currentLimelightState = TrackingState.HP_STATION;
  private SimPID headingPID;
  private SimPID xPID;
  private SimPID yPID;
  private double forward;
  private double strafe;
  private double azimuth;
  private LEDColourState desiredLedState = LEDColourState.OFF;
  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;
  private double headingTarget = 0;
  private Pose2d autoStartPose = new Pose2d(0, 0, new Rotation2d());

  private Pose2d autoAimTargetPose = new Pose2d(0, 0, new Rotation2d());

  private boolean targetingLeftStation = true;

  private int cycleBalanced = 0;

  private Drive() {
    super();
  }

  public enum DriveState {
    OUTPUT,
    ANGLE_LOCK,
    BALANCING,
    BALANCING_ANGLE_LOCK,
    HUMAN_AUTO_AIM,
    SCORING_AUTO_AIM
  };

  public enum TrackingState {
    HP_STATION,
    CUBES,
    CONES
  };

  public void setTrackingState(TrackingState newState) {
    this.currentLimelightState = newState;
  }

  public TrackingState getTrackingState() {
    return this.currentLimelightState;
  }

  public void setDriveState(DriveState newState) {
    this.currentDriveState = newState;
  }

  public DriveState getDriveState() {
    return this.currentDriveState;
  }

  public void setOutput(double iforward, double istrafe, double iazimuth) {
    this.forward = iforward;
    this.strafe = istrafe;
    this.azimuth = iazimuth;
  }

  public void setTargetAngle(double targetAngle) {
    this.headingTarget = targetAngle;
  }

  public void setTargetingLeftStation(boolean left) {
    this.targetingLeftStation = left;
  }

  public void setAutoStartPose(Pose2d pose) {
    this.autoStartPose = pose;
  }

  public Pose2d getAutoStartPose() {
    return this.autoStartPose;
  }

  public void setAutoAimTargetPose(Pose2d pose) {
    this.autoAimTargetPose = pose;
  }

  public static Drive getInstance() {
    if (instance == null) {
      instance = new Drive();
    }

    return instance;
  }

  public void firstCycle() {

    this.currentDriveState = DriveState.OUTPUT;
    this.headingPID = new SimPID(RobotConstants.getHeadingPID());

    this.headingPID.setIRange(2);

    this.xPID = new SimPID(RobotConstants.getDrivePID());
    this.yPID = new SimPID(RobotConstants.getDrivePID());

    this.xLimiter = new SlewRateLimiter(0.5);
    this.yLimiter = new SlewRateLimiter(0.5);
    this.turningLimiter = new SlewRateLimiter(4.0);

    this.yLimiter.calculate(0);
    this.xLimiter.calculate(0);
    this.turningLimiter.calculate(0);

    this.yLimiter.reset(0);
    this.xLimiter.reset(0);
    this.turningLimiter.reset(0);

    this.xPID.setIRange(0.15);
    this.yPID.setIRange(0.15);

    this.xPID.setMaxOutput(1.0);
    this.yPID.setMaxOutput(1.0);

    if (DriverStation.getAlliance() == Alliance.Red) {
      this.targetingLeftStation = true;
    } else {
      this.targetingLeftStation = false;
    }
  }

  public void calculate() {
    switch (this.currentDriveState) {
      case OUTPUT -> {
        this.desiredLedState = LEDColourState.OFF;
        this.io.drive(this.forward, this.strafe, this.azimuth);
        this.io.setFieldOriented(true);
        this.io.setDriveBrakeMode(false);
      }
      case BALANCING -> {
        // TODO, WHEN forward and strafe aren't pressed, lock wheels at 45
        this.desiredLedState = LEDColourState.OFF;
        this.io.drive(this.forward * 0.3, this.strafe * 0.3, this.azimuth * 0.3);
        this.io.setFieldOriented(true);
        this.io.setDriveBrakeMode(true);
      }
      case ANGLE_LOCK -> {
        this.headingPID.setDesiredValue(this.headingTarget);
        this.headingPID.enableContinuousInput(-180, 180);
        this.desiredLedState = LEDColourState.SOLID_BLUE;
        this.io.drive(this.forward, this.strafe, this.headingPID.calcPID(this.io.getHeading()));

        this.io.setFieldOriented(true);
        this.io.setDriveBrakeMode(false);
      }
      case BALANCING_ANGLE_LOCK -> {
        this.headingPID.setDesiredValue(this.headingTarget);
        this.headingPID.enableContinuousInput(-180, 180);
        this.desiredLedState = LEDColourState.SOLID_BLUE;
        this.io.drive(
            this.forward * 0.3, this.strafe * 0.3, this.headingPID.calcPID(this.io.getHeading()));
        this.io.setFieldOriented(true);
        this.io.setDriveBrakeMode(true);
      }
      case HUMAN_AUTO_AIM -> { // TODO
        this.headingPID.setDesiredValue(this.headingTarget);
        this.headingPID.enableContinuousInput(-180, 180);
        this.desiredLedState = LEDColourState.SOLID_BLUE;
        this.io.setFieldOriented(true);
        this.io.setDriveBrakeMode(true);

        if (this.targetingLeftStation) {
          if (DriverStation.getAlliance() == Alliance.Red) {
            this.io.drive(
                this.forward * 0.3,
                this.io.getRightLimelightX() * -0.01,
                this.headingPID.calcPID(this.io.getHeading()));
          } else {
            this.io.drive(
                this.forward * 0.3,
                this.io.getRightLimelightX() * 0.01,
                this.headingPID.calcPID(this.io.getHeading()));
          }
        } else {
          if (DriverStation.getAlliance() == Alliance.Red) {
            this.io.drive(
                this.forward * 0.3,
                this.io.getLeftLimelightX() * -0.01,
                this.headingPID.calcPID(this.io.getHeading()));
          } else {
            this.io.drive(
                this.forward * 0.3,
                this.io.getLeftLimelightX() * 0.01,
                this.headingPID.calcPID(this.io.getHeading()));
          }
        }
      }
      case SCORING_AUTO_AIM -> {
        this.io.setFieldOriented(true);
        this.io.setDriveBrakeMode(true);

        //  driveToPoint(this.autoAimTargetPose.getX(), this.autoAimTargetPose.getY(),
        //  this.autoAimTargetPose.getRotation().getDegrees(), 0, 0.6, 0.3, 0.01,false);
      }
    }

    SmartDashboard.putString("DRIVE STATE", this.currentDriveState.toString());
  }

  public boolean isStill() {
    return (Math.abs(this.io.getGyroSpeed()) < 0.1)
        && (Math.abs(this.io.getXVelocity()) < 0.05)
        && (Math.abs(this.io.getYVelocity()) < 0.05);
  }

  public LEDColourState getDesiredLedState() {
    return this.desiredLedState;
  }

  public void resetGyro() {
    this.io.zeroHeading();
    this.io.resetAutoStartAngle();
  }

  public void resetPose() {
    this.io.setDrivePose(0.0, 0.0, 0);
  }

  /**
   * Balances the robot using the navX
   *
   * @author navX/Team 2465
   * @param eps epsilon
   * @return if robot is balanced
   */
  public boolean balance(double eps, double theta) {
    double pitch = io.getGyroPitch();
    double roll = io.getGyroRoll();

    this.io.setFieldOriented(false);
    this.io.setDriveBrakeMode(true);
    this.headingPID.setMaxOutput(0.3);
    this.headingPID.setFinishedRange(0.5);
    this.headingPID.setIRange(2);
    this.headingPID.setDesiredValue(theta);
    this.headingPID.disableContinuousInput();

    // Thanks 2465/NavX code examples
    if (Math.abs(pitch) > eps || Math.abs(roll) > eps) {
      double xOutput = Math.sin(roll * (Math.PI / 180.0)) * -0.44;
      double yOutput = Math.sin(pitch * (Math.PI / 180.0)) * -0.44;
      xOutput =
          Math.max(
              Math.min(xOutput, RobotConstants.BALANCE_MAX_OUTPUT),
              -RobotConstants.BALANCE_MAX_OUTPUT);
      yOutput =
          Math.max(
              Math.min(yOutput, RobotConstants.BALANCE_MAX_OUTPUT),
              -RobotConstants.BALANCE_MAX_OUTPUT);
      if (DriverStation.getAlliance() == Alliance.Red) { // Is this different?
        io.drive(-xOutput, yOutput, this.headingPID.calcPID(this.io.getHeading()));
      } else {
        io.drive(-xOutput, yOutput, this.headingPID.calcPID(this.io.getHeading()));
      }

      this.cycleBalanced = 0;

    } else {
      io.drive(0, 0, this.headingPID.calcPID(this.io.getHeading()));
      this.cycleBalanced++;
    }

    return this.cycleBalanced > 150;
  }

  public Pose2d snapToClosestPosition(Pose2d home, Pose2d... other) {
    return home.nearest(List.of(other));
  }

  private SimPoint getRotatedError(double theta, double desiredX, double desiredY) {
    double currentX = this.io.getRobotPose().getX();
    double currentY = this.io.getRobotPose().getY();
    double rotation = theta;

    SimPoint currentPosition = new SimPoint(currentX, currentY);
    SimPoint finalPosition = new SimPoint(desiredX, desiredY);

    currentPosition.rotateByAngleDegrees(rotation);
    finalPosition.rotateByAngleDegrees(rotation);

    double xError = finalPosition.getX() - currentPosition.getX();
    double yError = finalPosition.getY() - currentPosition.getY();

    return new SimPoint(xError, yError);
  }

  public boolean driveToCube(double theta, double velocity, double turnSpeed, boolean right) {
    this.headingPID.setMaxOutput(turnSpeed);
    this.headingPID.setFinishedRange(0.5);
    this.headingPID.setIRange(2);
    this.headingPID.setDesiredValue(theta);
    this.headingPID.disableContinuousInput();
    this.io.setFieldOriented(false);

    double turningOutput = this.headingPID.calcPID(this.io.getHeading());

    this.io.setLimelightPipeline(2);

    if (right) { // Is this different? // make this red
      if (this.io.getRightLimelightValidTarget()) {
        this.io.drive(
            this.getCubeYVelocity(this.io.getRightLimelightX(), velocity),
            this.io.getRightLimelightX() * 0.025,
            turningOutput);
      } else if (this.io.getLeftLimelightValidTarget()) {
        this.io.drive(
            this.getCubeYVelocity(this.io.getLeftLimelightX(), velocity),
            this.io.getLeftLimelightX() * 0.025,
            turningOutput);
      } else {
        this.io.drive(velocity, 0.0, turningOutput);
      }

    } else {
      if (this.io.getLeftLimelightValidTarget()) {
        this.io.drive(
            this.getCubeYVelocity(this.io.getLeftLimelightX(), velocity),
            this.io.getLeftLimelightX() * 0.025,
            turningOutput);
      } else if (this.io.getRightLimelightValidTarget()) {
        this.io.drive(
            this.getCubeYVelocity(this.io.getRightLimelightX(), velocity),
            this.io.getRightLimelightX() * 0.025,
            turningOutput);
      } else {
        this.io.drive(velocity, 0.0, turningOutput);
      }
    }

    return false;
  }

  /**
   * @param xError
   * @return
   */
  private double getCubeYVelocity(double xError) {
    return getCubeYVelocity(xError, 0.1);
  }

  /**
   * @param xError
   * @param minVelocity
   * @return
   */
  private double getCubeYVelocity(double xError, double minVelocity) {
    return getCubeYVelocity(xError, minVelocity, 0.3);
  }

  /**
   * @param xError
   * @param minVelocity
   * @param scalar
   * @return
   */
  private double getCubeYVelocity(double xError, double minVelocity, double scalar) {
    if (Math.abs(xError) > 15) {
      return minVelocity;
    } else {
      // y = ((1 - x/15) * 0.4) + 0.1
      return (((1 - (Math.abs(xError) / 15.0)) * scalar) + minVelocity);
    }
  }

  public boolean driveToCubeAngle(double theta, double velocity, double turnSpeed) {
    this.headingPID.setMaxOutput(turnSpeed);
    this.headingPID.setFinishedRange(0.5);
    this.headingPID.setIRange(2);
    this.headingPID.setDesiredValue(theta);

    this.headingPID.disableContinuousInput();
    this.io.setFieldOriented(false);

    this.io.setLimelightPipeline(2);

    if (DriverStation.getAlliance() == Alliance.Red) { // Is this different?
      if (this.io.getRightLimelightValidTarget()) {
        this.headingPID.setDesiredValue(this.io.getHeading() + this.io.getRightLimelightX());
        double turningOutput = this.headingPID.calcPID(this.io.getHeading());
        if (Math.abs(this.io.getRightLimelightX()) < 15) {
          this.io.drive(velocity, 0, turningOutput);
        } else {
          this.io.drive(0, 0, turningOutput);
        }

      } else if (this.io.getLeftLimelightValidTarget()) {
        this.headingPID.setDesiredValue(this.io.getHeading() + this.io.getLeftLimelightX());
        double turningOutput = this.headingPID.calcPID(this.io.getHeading());
        if (Math.abs(this.io.getLeftLimelightX()) < 15) {
          this.io.drive(velocity, 0, turningOutput);
        } else {
          this.io.drive(0, 0, turningOutput);
        }
      } else {
        double turningOutput = this.headingPID.calcPID(this.io.getHeading());
        this.io.drive(0, 0.0, turningOutput);
      }

    } else {
      if (this.io.getLeftLimelightValidTarget()) {
        this.headingPID.setDesiredValue(this.io.getHeading() + this.io.getLeftLimelightX());
        double turningOutput = this.headingPID.calcPID(this.io.getHeading());
        if (Math.abs(this.io.getLeftLimelightX()) < 15) {
          this.io.drive(velocity, 0, turningOutput);
        } else {
          this.io.drive(0, 0, turningOutput);
        }
      } else if (this.io.getRightLimelightValidTarget()) {
        this.headingPID.setDesiredValue(this.io.getHeading() + this.io.getRightLimelightX());
        double turningOutput = this.headingPID.calcPID(this.io.getHeading());
        if (Math.abs(this.io.getRightLimelightX()) < 15) {
          this.io.drive(velocity, 0, turningOutput);
        } else {
          this.io.drive(0, 0, turningOutput);
        }
      } else {
        double turningOutput = this.headingPID.calcPID(this.io.getHeading());
        this.io.drive(0, 0.0, turningOutput);
      }
    }

    return false;
  }

  public boolean driveToPoint(
      double x,
      double y,
      double theta,
      double minVelocity,
      double maxVelocity,
      double turnSpeed,
      double eps,
      boolean autoOriginRelative) {

    this.headingPID.setMaxOutput(turnSpeed);
    this.headingPID.setFinishedRange(0.5);
    this.headingPID.setIRange(2);
    this.headingPID.setDesiredValue(theta);
    this.xPID.setFinishedRange(eps);
    this.yPID.setFinishedRange(eps);
    this.io.setFieldOriented(true);

    if (this.io.getIsAuto() && autoOriginRelative) {
      x += this.autoStartPose.getX(); // offset the auto commands by the starting location
      y += this.autoStartPose.getY();
      this.headingPID.disableContinuousInput();
    } else {
      if (this.io.getIsAuto()) {
        this.headingPID.disableContinuousInput();
      } else {
        this.headingPID.enableContinuousInput(-180, 180);
      }
    }

    double yError = (y - (this.io.getRobotPose()).getY());
    double xError = x - (this.io.getRobotPose()).getX();

    Vect vectError = new Vect(xError, yError);

    double dist = vectError.mag();

    vectError = vectError.unit();

    vectError = vectError.scalarMult(maxVelocity);

    this.yPID.setMaxOutput(
        Math.abs(vectError.getY())); // scales so always at least 1 is going the max velocity
    this.xPID.setMaxOutput(Math.abs(vectError.getX()));

    double yOutput = this.yPID.calcPIDError(yError);
    double xOutput = this.xPID.calcPIDError(xError);

    Vect vectOutput = new Vect(xOutput, yOutput);

    if (vectOutput.mag() < minVelocity) { // if the overall vector speed is less than the minimum
      vectOutput = vectOutput.unit(); // get the unit vector

      vectOutput = vectOutput.scalarMult(minVelocity); // scale it to the min
    }

    yOutput = vectOutput.getY();
    xOutput = vectOutput.getX();
    double turningOutput = this.headingPID.calcPID(this.io.getHeading());

    double yRamp = this.yLimiter.calculate(yOutput);
    double xRamp = this.xLimiter.calculate(xOutput);
    double turningRamp = this.turningLimiter.calculate(turningOutput);

    if (dist > 0.3) { // ramp when far away
      yOutput = yRamp;
      xOutput = xRamp;
    }

    if (Math.abs(this.headingPID.getDesiredVal() - this.io.getHeading()) > 20) {
      turningOutput = turningRamp;
    }

    boolean isDone = false;
    if (minVelocity <= 0.01) {
      if (this.xPID.isDone() && this.yPID.isDone()) {

        isDone = true;
        this.io.drive(0, 0, 0);
        this.turningLimiter.reset(0);
        this.xLimiter.reset(0);
        this.yLimiter.reset(0);
        // slew ramp reset
      } else { // not done so drive
        this.io.drive(xOutput, yOutput, turningOutput);
      }
    } else if (dist < eps) {
      isDone = true;
    } else {
      this.io.drive(xOutput, yOutput, turningOutput);
    }

    return isDone;
  }

  public void resetRateLimit() {
    this.yLimiter.reset(0);
    this.xLimiter.reset(0);
    this.turningLimiter.reset(0);
  }

  public void disable() {
    this.io.drive(0, 0, 0);
  }
}
