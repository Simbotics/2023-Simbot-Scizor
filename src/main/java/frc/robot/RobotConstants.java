package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.io.Dashboard;
import frc.util.PIDConstants;

public class RobotConstants {

  public static final boolean USING_DASHBOARD = false;
  public static final boolean TUNING_PID = false;
  public static final boolean USING_LIMELIGHTS = true;

  // Drive Size Constants (Feet)
  public static final double DRIVE_MAX_VELOCITY = 18.5;
  public static final double DRIVE_TICKS_PER_REV = 2048.0;

  public static final double CANCODER_TICKS_PER_REV = 4096.0;

  public static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(28.5); // INCLUDES BUMPERS
  public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(31.5); // INCLUDES BUMPERS

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters =
        Units.inchesToMeters((4.0 * 1.0789) * (10.5 / 10.0) * (4.0 / 4.50594));
    public static final double kDriveMotorGearRatio =
        1.0 / ((40.0 / 14.0) * (44.0 / 40.0) * (18.0 / 28.0) * (45.0 / 15.0));
    public static final double kTurningMotorGearRatio = 1.0 / ((24.0 / 8.0) * (72.0 / 14.0));
    public static final double kDriveEncoderRot2Meter =
        (2048.0 / kDriveMotorGearRatio) * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = (2048.0 / kTurningMotorGearRatio) / 2.0;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
  }

  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(17.068);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(20.068);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0));

    public static final int kFrontLeftDriveMotorPort = 7;
    public static final int kBackLeftDriveMotorPort = 9;
    public static final int kFrontRightDriveMotorPort = 8;
    public static final int kBackRightDriveMotorPort = 10;

    public static final int kFrontLeftTurningMotorPort = 3;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackRightTurningMotorPort = 6;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
    public static final int kBackRightDriveAbsoluteEncoderPort = 2;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kBackLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kBackRightDriveMotorReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =
        -0.675 - 0.06 - 3.6 + 2.9 + 0.06 + 0.238 - 0.137 + 0.13; // Math.PI + 0.937262;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad =
        2.83 - 6.81 + 0.4; // 0.276117;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =
        2.10 - 4.79 + 0.005 - 0.01 - 0.19 - 0.89 + 2.273 + 0.13; // - 1.965;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad =
        -3.88 + 0.05 - 1.07 - 0.28 + 1.6 + 0.18; // Math.PI + 2.816922;

    public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(18.5);
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  // DRIVE CONSTANTS

  public static final double BALANCE_MAX_OUTPUT = 0.3;

  public static final double VISION_SCORING_AREA_CUTOFF = 0.3;
  public static final double VISION_HUMAN_LOAD_AREA_CUTOFF = 0.18;

  // INTAKE CONSTANTS

  public static final double CLAW_INTAKING_ANGLE = 40;
  public static final double INTAKE_CLOSED_CONE_PID_VALUE = 0;

  /** ARM CONSTANTS. */
  public static final double DISTAL_ARM_SCORING_THRESHOLD = 70;

  public static final double PROXIMAL_ARM_SCORING_THRESHOLD = -2;

  public static final double DISTAL_ARM_INTAKING_CUBE_THRESHOLD = 25;

  public static final double INITIAL_PROXIMAL_ARM_ANGLE =
      110.8 - 28.69 - 45.0 + 27.8 - 1.5 - 0.23 + 8.106 + 5.4 + 90.0 - 61.704 - 5.1;
  public static final double INITIAL_DISTAL_ARM_ANGLE = 89.8 + 3.1 - 2 + 3.8 - 0.29 - 0.5 + 35.0;

  // public static final double INITIAL_WRIST_ANGLE = -173.8 + 49.6 - 28.5 - 40.9 - 8.2;
  public static final double INITIAL_WRIST_ANGLE = -173.8 - 1.0 + 85.0 - 80.5 - 2.5 - 5.0 + 6.0;

  public static final double PROXIMAL_ARM_ENCODER_RATIO = 65.0 / 40.0;
  public static final double DISTAL_ARM_ENCODER_RATIO = 42.0 / 35.0;

  public static final double WRIST_ENCODER_RATIO = 80.0 / 36.0;
  public static final double ROLLER_AMPS = 16.0;

  // Holding angles.
  public static final double PROXIMAL_ARM_HOLDING_ANGLE = -11.0;
  public static final double DISTAL_ARM_HOLDING_ANGLE = 5.0;

  // Human loading station angles.
  public static final double PROXIMAL_ARM_HUMAN_LOADING_STATION_ANGLE = 28.5;
  public static final double DISTAL_ARM_HUMAN_LOADING_STATION_ANGLE = 90.25;

  // Intaking angles.
  public static final double PROXIMAL_ARM_INTAKING_CONE_ANGLE = 34.5;
  public static final double DISTAL_ARM_INTAKING_CONE_ANGLE = -1.5;

  public static final double PROXIMAL_ARM_INTAKING_CONE_TIPPED_ANGLE = 34.5;
  public static final double DISTAL_ARM_INTAKING_CONE_TIPPED_ANGLE = -1.5;

  public static final double PROXIMAL_ARM_INTAKING_CUBE_ANGLE = -11.0;
  public static final double DISTAL_ARM_INTAKING_CUBE_ANGLE = 34.25;

  // Low scoring angles.
  public static final double PROXIMAL_ARM_SCORING_LOW_CONE_ANGLE = -11.0;
  public static final double DISTAL_ARM_SCORING_LOW_CONE_ANGLE = 18.0;

  public static final double PROXIMAL_ARM_SCORING_LOW_CUBE_ANGLE = -11.0;
  public static final double DISTAL_ARM_SCORING_LOW_CUBE_ANGLE = 5.0;

  // Medium scoring angles.
  public static final double PROXIMAL_ARM_SCORING_MEDIUM_CONE_ANGLE = 1.0; // -11
  public static final double DISTAL_ARM_SCORING_MEDIUM_CONE_ANGLE = 89.0; // 78

  public static final double PROXIMAL_ARM_SCORING_MEDIUM_CONE_TIPPED_ANGLE = 2.0; // -11
  public static final double DISTAL_ARM_SCORING_MEDIUM_CONE_TIPPED_ANGLE = 97.5; // 78

  public static final double PROXIMAL_ARM_SCORING_MEDIUM_CUBE_ANGLE = -11.0;
  public static final double DISTAL_ARM_SCORING_MEDIUM_CUBE_ANGLE = 73.0;

  // High scoring angles.
  public static final double PROXIMAL_ARM_SCORING_HIGH_CONE_ANGLE = 25.0;
  public static final double DISTAL_ARM_SCORING_HIGH_CONE_ANGLE = 103.0;

  public static final double PROXIMAL_ARM_SCORING_HIGH_CUBE_ANGLE = -11.0;
  public static final double DISTAL_ARM_SCORING_HIGH_CUBE_ANGLE = 84.0;

  public static final double PROXIMAL_ARM_GRAVITY_OFFSET = 0.00;
  public static final double DISTAL_ARM_GRAVITY_OFFSET = 0.025;

  // Wrist intaking angles;
  public static final double HOLDING_ANGLE = 100.0;
  public static final double INTAKING_CONE_ANGLE = 74.0;
  public static final double INTAKING_CONE_TIPPED_ANGLE = 74.0;
  public static final double INTAKING_CUBE_ANGLE = 34.0;
  public static final double INTAKING_HUMAN_LOADING_ANGLE = 90.0;
  public static final double SCORING_LOW_ANGLE = 100.0;
  public static final double SCORING_CONE_MID_ANGLE = 87.0; // 130
  public static final double SCORING_CONE_MID_TIPPED_ANGLE = 26.0; // 130
  public static final double SCORING_CUBE_MID_ANGLE = 86.0;
  public static final double SCORING_CONE_HIGH_ANGLE = 130.0;
  public static final double SCORING_CUBE_HIGH_ANGLE = 125.0;

  // Wrist thresholds
  public static final double WRIST_HOLDING_THRESHOLD = 72.0;

  public static final String LIMELIGHT_LEFT = "limelight-left";
  public static final String LIMELIGHT_RIGHT = "limelight-right";
  public static final String LIMELIGHT_BACK = "limelight-back";

  /** PID constants. */
  private static Dashboard dashboard = Dashboard.getInstance();

  private static PIDConstants modulePID = new PIDConstants(0.5, 0.0100, 0.0, 0.0);

  private static PIDConstants headingPID = new PIDConstants(0.014, 0.0004, 0.05, 0.0);
  private static PIDConstants drivePID = new PIDConstants(0.56, 0.00045, 0.001, 0.01);

  private static PIDConstants wristPID = new PIDConstants(0.013, 0, 0, 1.5);

  private static PIDConstants distalArmUpPID = new PIDConstants(0.02, 0, 0, 0);
  private static PIDConstants distalArmDownPID = new PIDConstants(0.013, 0, 0, 0);
  private static PIDConstants proximalArmPID = new PIDConstants(0.030, 0.0002, 0.05, 0);

  // Camera Constants

  public static PIDConstants getModulePID() {
    if (USING_DASHBOARD) {
      return dashboard.getPIDConstants("MODULE_PID", modulePID);
    } else {
      return modulePID;
    }
  }

  public static PIDConstants getHeadingPID() {
    if (USING_DASHBOARD) {
      return dashboard.getPIDConstants("HEADING_PID", headingPID);
    } else {
      return headingPID;
    }
  }

  public static PIDConstants getDrivePID() {
    if (USING_DASHBOARD) {
      return dashboard.getPIDConstants("DRIVE_PID", drivePID);
    } else {
      return drivePID;
    }
  }

  public static PIDConstants getWristPID() {
    if (USING_DASHBOARD) {
      return dashboard.getPIDConstants("WRIST_PID", wristPID);
    } else {
      return wristPID;
    }
  }

  /** Arm PIDs. */

  // yeah

  public static PIDConstants getDistalArmUpPID() {
    if (USING_DASHBOARD) {
      return dashboard.getPIDConstants("DISTAL_ARM_UP_PID", distalArmUpPID);
    } else {
      return distalArmUpPID;
    }
  }

  public static PIDConstants getDistalArmDownPID() {
    if (USING_DASHBOARD) {
      return dashboard.getPIDConstants("DISTAL_ARM_DOWN_PID", distalArmDownPID);
    } else {
      return distalArmDownPID;
    }
  }

  public static PIDConstants getProximalArmPID() {
    if (USING_DASHBOARD) {
      return dashboard.getPIDConstants("PROXIMAL_ARM_PID", proximalArmPID);
    } else {
      return proximalArmPID;
    }
  }

  // Pushes the values to the smart dashboard
  public static void pushValues() {

    // dashboard.putPIDConstants("TOP_SHOOTER_PID", shooterTopPID);
    // dashboard.putPIDConstants("BOTTOM_SHOOTER_PID", shooterBottomPID);
    // dashboard.putPIDConstants("HEADING_PID", headingPID);
    //  dashboard.putPIDConstants("MODULE_PID", modulePID);
    // dashboard.putPIDConstants("INDEXER_PID", indexerPID);
    // dashboard.putPIDConstants("DRIVE_PID", drivePID);
    // dashboard.putPIDConstants("WRIST_PID", wristPID);
    // dashboard.putPIDConstants("DISTAL_ARM_UP_PID", distalArmUpPID);
    // dashboard.putPIDConstants("DISTAL_ARM_DOWN_PID", distalArmDownPID);
    // dashboard.putPIDConstants("PROXIMAL_ARM_PID", proximalArmPID);
  }
}
