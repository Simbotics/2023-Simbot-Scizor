package frc.io;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriveConstants;
import frc.util.Vect;
import frc.util.swerve.SwerveDrive;

public class IO {

  private static IO instance;

  // Sensors
  private PowerDistribution pdp;

  private double lastTime = 0.0;
  private double deltaTime = 10.0;

  private boolean firstCycle = true;

  // Match details.
  private DriverStation.Alliance alliance;
  private double matchTime;

  private long SystemTimeAtAutonStart = 0;
  private long timeSinceAutonStarted = 0;

  private boolean isAuto = false;

  private Vect lastPose = new Vect(0, 0);
  private double xVelocity = 0;
  private double yVelocity = 0;

  // Swerve turning and drive motor ID's.
  private int[] drivingPorts;
  private int[] steeringPorts;

  private SwerveDrive swerveDrive;

  private CANSparkMax wristMotor;
  private CANCoder wristEncoder;

  private VictorSP rollerMotor;

  private int rollerCurrentCheckCycles = 0;
  private double lastRollerCurrent = 0;

  // Arm encoders.
  private CANCoder distalArmEncoder;
  private CANCoder proximalArmEncoder;

  // Arm motors.
  private CANSparkMax LeftDistalArm;
  private CANSparkMax RightDistalArm;

  private CANSparkMax LeftProximalArm;
  private CANSparkMax RightProximalArm;

  private double proximalArmRampRate;
  private double distalArmRampRate;
  private double wristRampRate;

  private Field2d dashboardField;

  private SlewRateLimiter intakeRamp;

  private IO() {
    this.pdp = new PowerDistribution();

    int[] steeringPorts =
        new int[] {
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          DriveConstants.kBackRightTurningMotorPort
        };
    int[] drivingPorts =
        new int[] {
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackRightDriveMotorPort
        };
    int[] encoderPorts =
        new int[] {
          DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
          DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
          DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
          DriveConstants.kBackRightDriveAbsoluteEncoderPort
        };
    boolean[] drivingReversed =
        new boolean[] {
          DriveConstants.kFrontLeftDriveMotorReversed,
          DriveConstants.kFrontRightDriveMotorReversed,
          DriveConstants.kBackLeftDriveMotorReversed,
          DriveConstants.kBackRightDriveMotorReversed
        };
    boolean[] turningReversed =
        new boolean[] {
          DriveConstants.kFrontLeftTurningEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          DriveConstants.kBackLeftTurningEncoderReversed,
          DriveConstants.kBackRightTurningEncoderReversed
        };
    double[] moduleOffsets =
        new double[] {
          DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad
        };
    boolean[] absoluteReversed =
        new boolean[] {
          DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
          DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
          DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
          DriveConstants.kBackRightDriveAbsoluteEncoderReversed
        };

    // Server drive motors
    this.swerveDrive =
        new SwerveDrive(
            steeringPorts,
            drivingPorts,
            encoderPorts,
            drivingReversed,
            turningReversed,
            moduleOffsets,
            absoluteReversed);

    this.dashboardField = new Field2d();

    // Arm motors
    this.LeftProximalArm = new CANSparkMax(11, MotorType.kBrushless);
    this.RightProximalArm = new CANSparkMax(12, MotorType.kBrushless);

    this.LeftDistalArm = new CANSparkMax(13, MotorType.kBrushless);
    this.RightDistalArm = new CANSparkMax(14, MotorType.kBrushless);

    this.proximalArmEncoder = new CANCoder(18);
    this.distalArmEncoder = new CANCoder(19);

    // Claw motor and encoder
    this.wristMotor = new CANSparkMax(15, MotorType.kBrushless);
    this.wristEncoder = new CANCoder(20);

    this.rollerMotor = new VictorSP(1);

    this.intakeRamp = new SlewRateLimiter(1.25, -7.0, 0);

    this.configureSpeedControllers();

    this.reset();
  }

  /**
   * Gets the current IO instance.
   *
   * @return The IO instance if it isn't null.
   */
  public static IO getInstance() {
    if (instance == null) {
      instance = new IO();
    }
    return instance;
  }

  /** Resets the cycle back to the first cycle. */
  public void reset() {
    this.firstCycle = true;
  }

  /** Configures motors/speed controllers. */
  public void configureSpeedControllers() {

    // Wrist
    this.wristMotor.setIdleMode(IdleMode.kBrake);

    // Arms
    this.LeftProximalArm.setIdleMode(IdleMode.kBrake);
    this.RightProximalArm.setIdleMode(IdleMode.kBrake);

    this.LeftDistalArm.setIdleMode(IdleMode.kBrake);
    this.RightDistalArm.setIdleMode(IdleMode.kBrake);

    this.RightProximalArm.follow(LeftProximalArm, true);
    this.RightDistalArm.follow(LeftDistalArm, false);

    this.LeftDistalArm.enableVoltageCompensation(10.0);
    this.LeftProximalArm.enableVoltageCompensation(10.0);

    this.LeftProximalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 300);
    this.RightProximalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 300);
    this.LeftDistalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 300);
    this.RightDistalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 300);

    this.LeftProximalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 300);
    this.RightProximalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 300);
    this.LeftDistalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 300);
    this.RightDistalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 300);

    this.LeftProximalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 300);
    this.RightProximalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 300);
    this.LeftDistalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 300);
    this.RightDistalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 300);

    this.LeftProximalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 300);
    this.RightProximalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 300);
    this.LeftDistalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 300);
    this.RightDistalArm.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 300);
  }

  public void update() {

    // Update delta time.
    if (this.lastTime == 0.0) {
      this.deltaTime = 10;
      this.lastTime = System.currentTimeMillis();
    } else {
      this.deltaTime = System.currentTimeMillis() - this.lastTime;
      this.lastTime = System.currentTimeMillis();
    }

    // Prints time since auto started to smartdashboard.
    if (DriverStation.isAutonomous()) {
      this.timeSinceAutonStarted = System.currentTimeMillis() - this.SystemTimeAtAutonStart;
      SmartDashboard.putNumber("12_Time Since Auto Start:", this.timeSinceAutonStarted);
    }

    // If it's the first cycle make it not the first cycle.
    if (this.firstCycle) {
      this.firstCycle = false;
    }

    // Match details.
    this.alliance = DriverStation.getAlliance();
    this.matchTime = DriverStation.getMatchTime();

    this.swerveDrive.updateOdometry();

    // X and Y velocities in meters per second.
    this.xVelocity =
        (this.swerveDrive.getPose().getX() - this.lastPose.getX()) * (1000.0 / deltaTime);
    this.yVelocity =
        (this.swerveDrive.getPose().getY() - this.lastPose.getY()) * (1000.0 / deltaTime);

    this.lastPose = new Vect(this.swerveDrive.getPose().getX(), this.swerveDrive.getPose().getY());
    this.dashboardField.setRobotPose(getRobotPose());

    // Positional indications.
    // SmartDashboard.putNumber("X VELOCITY", this.xVelocity);
    // SmartDashboard.putNumber("Y VELOCITY", this.yVelocity);
    // SmartDashboard.putNumber("X Position", this.swerveDrive.getX());
    // SmartDashboard.putNumber("Y Position", this.swerveDrive.getY());

    SmartDashboard.putString("ALLIANCE: ", this.alliance.toString());

    // Claw indications.
    SmartDashboard.putNumber("WRIST ENCODER", this.getWristEncoder());
    SmartDashboard.putNumber("WRIST VELOCITY", this.getWristVelocity());

    // Arm indications.
    SmartDashboard.putNumber("DISTAL ARM ENCODER", this.getDistalArmEncoder());
    SmartDashboard.putNumber("PROXIMAL ARM ENCODER", this.getProximalArmEncoder());

    // Arm angles.
    SmartDashboard.putNumber("DISTAL ARM ANGLE", this.getDistalArmAngle());
    SmartDashboard.putNumber("PROXIMAL ARM ANGLE", this.getProximalArmAngle());
    SmartDashboard.putNumber("WRIST ARM ANGLE", this.getWristAngle());

    SmartDashboard.putNumber("GYRO HEADING", this.getHeading());
    //  SmartDashboard.putNumber("GYRO HEADING Continuous", this.getAbsoluteHeading());
    // SmartDashboard.putNumber("GYRO SPEED", this.getGyroSpeed());

    SmartDashboard.putNumber("GYRO PITCH", this.getGyroPitch());
    SmartDashboard.putNumber("GYRO ROLL", this.getGyroRoll());

    //   SmartDashboard.putNumber("ROLLER CURRENT AMPS", this.getRollerCurrent());

    SmartDashboard.putData("FIELD", dashboardField);

    if (this.rollerCurrentCheckCycles % 3 == 0) {
      this.lastRollerCurrent = this.pdp.getCurrent(14);
    }

    this.rollerCurrentCheckCycles++;
  }
  /** Claw and arm motor and encoder methods */

  /** Resets the wrist encoder rotations back to 0. */
  public void resetWristEncoder() {
    // this.wristEncoder.setPosition(0);
  }

  public void setClawEncoderPosition(double degrees) {
    double position = ((degrees / 360.0) * RobotConstants.WRIST_ENCODER_RATIO);
    this.wristEncoder.setPosition(position);
  }

  /**
   * Gets the number of rotations the roller motor has completed.
   *
   * @return The number of rotations of the roller motor.
   */
  public double getWristEncoder() {
    return -this.wristEncoder.getAbsolutePosition();
  }

  /**
   * Gets the number of rotations the roller motor has completed.
   *
   * @return The number of rotations of the roller motor.
   */
  public double getWristVelocity() {
    return this.wristEncoder.getVelocity();
  }

  public double getRollerCurrent() {
    return this.lastRollerCurrent;
  }

  /** Resets the distal arm encoder rotations back to 0. */
  public void resetDistalArmEncoder() {
    this.distalArmEncoder.setPosition(0);
  }

  public double getDistalArmEncoder() {
    return this.distalArmEncoder.getAbsolutePosition();
  }

  public void resetProximalArmEncoder() {
    this.proximalArmEncoder.setPosition(0);
  }

  public double getProximalArmEncoder() {
    return this.proximalArmEncoder.getAbsolutePosition();
  }

  /**
   * Sets the speed of the wrist motor.
   *
   * @param output Speed to set the motor to.
   */
  public void setWristMotor(double output) {
    this.wristMotor.set(output);
  }

  /**
   * Sets the speed of the wrist motor.
   *
   * @param output Speed to set the motor to.
   */
  public void setRollerMotor(double output) {

    this.rollerMotor.set(-this.intakeRamp.calculate(output));
  }

  /**
   * Sets the speed of the distal arm.
   *
   * @param output Speed of the distal arm.
   */
  public void setDistalArm(double output) {
    // System.out.println("DISTAL OUTPUT : " + output);
    this.LeftDistalArm.set(output);
  }

  /**
   * Sets the speed of the proximal arm.
   *
   * @param output Speed of the proximal arm.
   */
  public void setProximalArm(double output) {
    this.LeftProximalArm.set(-output);
  }

  /**
   * Gets the proximal arms angle.
   *
   * @return The proximal arms angle in degrees as a double.
   */
  public double getProximalArmAngle() {
    return (this.getProximalArmEncoder() / RobotConstants.PROXIMAL_ARM_ENCODER_RATIO)
        - RobotConstants.INITIAL_PROXIMAL_ARM_ANGLE;
  }

  /**
   * Gets the distal arms angle.
   *
   * @return The distal arms angle in degrees as a double.
   */
  public double getDistalArmAngle() {
    return (this.getDistalArmEncoder() / RobotConstants.DISTAL_ARM_ENCODER_RATIO)
        - RobotConstants.INITIAL_DISTAL_ARM_ANGLE;
  }

  /**
   * Gets the distal arms angle.
   *
   * @return The distal arms angle in degrees as a double.
   */
  public double getWristAngle() {
    return ((this.getWristEncoder() / RobotConstants.WRIST_ENCODER_RATIO)
        - RobotConstants.INITIAL_WRIST_ANGLE);
  }

  /**
   * Sets the proximal arm's ramp rate.
   *
   * @param rampRate Ramp rate you want to set it to.
   */
  public void setProximalArmRampRate(double rampRate) {
    if (Math.abs(this.proximalArmRampRate - rampRate) > 0.01) {
      this.proximalArmRampRate = rampRate;
      this.LeftProximalArm.setOpenLoopRampRate(rampRate);
    }
  }

  /**
   * Sets the distal arm's ramp rate.
   *
   * @param rampRate Ramp rate you want to set it to.
   */
  public void setDistalArmRampRate(double rampRate) {
    if (Math.abs(this.distalArmRampRate - rampRate) > 0.01) {
      this.distalArmRampRate = rampRate;
      this.LeftDistalArm.setOpenLoopRampRate(rampRate);
    }
  }

  /**
   * Gets the ramining match time left.
   *
   * @return The remaining match time.
   */
  public double getMatchTimeLeft() {
    return this.matchTime;
  }

  /** Resets the time since auto started time. */
  public void resetAutonTimer() {
    this.SystemTimeAtAutonStart = System.currentTimeMillis();
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
  }

  public boolean getIsAuto() {
    return this.isAuto;
  }

  /** PDP methods. */

  /**
   * Gets the voltage that the PDP is recognizing.
   *
   * @return The voltage.
   */
  public double getVoltage() {
    return this.pdp.getVoltage();
  }

  /**
   * Gets the amount of current in a secific PDP port.
   *
   * @param port The port to get the current of.
   * @return The recognized current for the port.
   */
  public double getCurrent(int port) {
    return this.pdp.getCurrent(port);
  }

  public double getDeltaTime() {
    return this.deltaTime;
  }

  /** Drive methods. */

  /**
   * Gets the swerve drive turning motor ID's.
   *
   * @return
   */
  public int[] getSteeringPorts() {
    return this.steeringPorts;
  }

  /**
   * Gets the swerve drive driving motor ID's.
   *
   * @return
   */
  public int[] getDrivingPorts() {
    return this.drivingPorts;
  }

  public void setVisionAreaCutoff(double cutoff) {
    this.swerveDrive.setVisionAreaCutoff(cutoff);
  }

  public void setOdometryUsingLimelight(boolean usingLimelights) {
    this.swerveDrive.setUsingLimelights(usingLimelights);
  }

  public double getLeftLimelightX() {
    return this.swerveDrive.getLeftLimelightX();
  }

  public double getRightLimelightX() {
    return this.swerveDrive.getRightLimelightX();
  }

  public boolean getLeftLimelightValidTarget() {
    return this.swerveDrive.getLeftLimelightValidTarget();
  }

  public boolean getRightLimelightValidTarget() {
    return this.swerveDrive.getRightLimelightValidTarget();
  }

  public void setLimelightPipeline(int pipeline) {
    this.swerveDrive.setLimelightPipeline(pipeline);
  }

  public void drive(double xSpeed, double ySpeed, double turningSpeed) {
    this.swerveDrive.drive(xSpeed, ySpeed, turningSpeed);
  }

  public void setFieldOriented(boolean fieldOriented) {
    this.swerveDrive.setFieldOriented(fieldOriented);
  }

  public void setDriveBrakeMode(boolean brake) {
    this.swerveDrive.setBrakeMode(brake);
  }

  public double getHeading() {
    return this.swerveDrive.getHeading();
  }

  public double getAbsoluteHeading() {
    return this.swerveDrive.getAbsoluteHeading();
  }

  public double getGyroSpeed() {
    return this.swerveDrive.getGyroSpeed();
  }

  public double getGyroPitch() {
    return this.swerveDrive.getPitch();
  }

  public double getGyroRoll() {
    return this.swerveDrive.getRoll();
  }

  public void zeroHeading() {
    this.swerveDrive.zeroHeading();
  }

  public double getXVelocity() {
    return this.xVelocity;
  }

  public double getYVelocity() {
    return this.yVelocity;
  }

  public Pose2d getRobotPose() {
    return (this.swerveDrive.getPose());
  }

  public void setDrivePose(double x, double y, double theta) {
    this.swerveDrive.zeroHeading();
    this.swerveDrive.setAutoStartAngle(theta);
    this.swerveDrive.resetOdometry(x, y, theta);
  }

  /** Resets the starting auto angle back to 0 degrees. */
  public void resetAutoStartAngle() {
    this.swerveDrive.setAutoStartAngle(0);
  }

  /** Shuts off components. */
  public void stopAll() {
    // shut off things here
  }
}
