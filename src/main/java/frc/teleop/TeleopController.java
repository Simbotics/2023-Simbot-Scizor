package frc.teleop;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.io.DriverInput;
import frc.io.IO;
import frc.robot.RobotConstants;
import frc.subsystems.Arm;
import frc.subsystems.Arm.ArmState;
import frc.subsystems.Arm.WristState;
import frc.subsystems.Drive;
import frc.subsystems.Drive.DriveState;
import frc.subsystems.Intake;
import frc.subsystems.Intake.RollerState;
import frc.subsystems.LEDStrip;
import frc.subsystems.LEDStrip.LEDColourState;
import frc.util.SimLib;
import java.util.List;

public class TeleopController extends TeleopComponent {

  private static TeleopController instance;
  private DriverInput driverIn;

  private Drive drive;
  private Intake intake;
  private Arm arm;
  private IO io;
  private LEDStrip ledstrips;

  private int scoringCycles = 0;

  private boolean driverManual = false;
  private boolean operatorManual = false;
  private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public static TeleopController getInstance() {
    if (instance == null) {
      instance = new TeleopController();
    }
    return instance;
  }

  private TeleopController() {

    this.driverIn = DriverInput.getInstance();

    this.intake = Intake.getInstance();
    this.drive = Drive.getInstance();

    this.ledstrips = LEDStrip.getInstance();

    this.arm = Arm.getInstance();

    this.io = IO.getInstance();

    this.xLimiter = new SlewRateLimiter(3);
    this.yLimiter = new SlewRateLimiter(3);
    this.turningLimiter = new SlewRateLimiter(3);
  }

  @Override
  public void firstCycle() {
    this.drive.firstCycle();
    this.arm.firstCycle();
    this.intake.firstCycle();
  }

  @Override
  public void calculate() {

    boolean isBalancing =
        this.drive.getDriveState() == DriveState.BALANCING
            || this.drive.getDriveState() == DriveState.BALANCING_ANGLE_LOCK;

    boolean isScoring =
        this.intake.getRollerState() == RollerState.SCORING_MID_CONE
            || this.intake.getRollerState() == RollerState.SCORING_MID_CUBE
            || this.intake.getRollerState() == RollerState.SCORING_HIGH_CONE
            || this.intake.getRollerState() == RollerState.SCORING_MID_CONE_TIPPED
            || this.intake.getRollerState() == RollerState.SCORING_HIGH_CUBE
            || this.intake.getRollerState() == RollerState.SCORING_LOW;

    if (this.driverIn.getDriverManualOnButton()) {
      this.driverManual = true;
    } else if (this.driverIn.getDriverManualOffButton()) {
      this.driverManual = false;
    }

    // Drive
    double leftY = SimLib.squareMaintainSign(this.driverIn.getDriverLeftY());
    double leftX = SimLib.squareMaintainSign(this.driverIn.getDriverLeftX());
    if (DriverStation.getAlliance() == Alliance.Red) {
      leftX *= -1.0;
      leftY *= -1.0;
    }

    double rightX = SimLib.squareMaintainSign(this.driverIn.getDriverRightX());

    if (this.driverIn.getFaceStationButton()) {
      if (isBalancing) {
        this.drive.setDriveState(DriveState.BALANCING_ANGLE_LOCK);
      } else {
        this.drive.setDriveState(DriveState.ANGLE_LOCK);
      }
      if (DriverStation.getAlliance() == Alliance.Red) {
        this.drive.setTargetAngle(180);
      } else {
        this.drive.setTargetAngle(0);
      }

    } else if (this.driverIn.getFaceGridButton()) {
      if (isBalancing) {
        this.drive.setDriveState(DriveState.BALANCING_ANGLE_LOCK);
      } else {
        this.drive.setDriveState(DriveState.ANGLE_LOCK);
      }
      if (DriverStation.getAlliance() == Alliance.Red) {
        this.drive.setTargetAngle(0);
      } else {
        this.drive.setTargetAngle(180);
      }
    } else if (this.driverIn.getDriverAutoPilotButton()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        this.drive.setTargetAngle(180);
      } else {
        this.drive.setTargetAngle(0);
      }

      this.io.setLimelightPipeline(1);
      this.drive.setDriveState(DriveState.HUMAN_AUTO_AIM); // loading station
      this.io.setOdometryUsingLimelight(false);

    } else if (this.driverIn.getBalanceButton()) {
      if (this.drive.getDriveState() == DriveState.ANGLE_LOCK
          || this.drive.getDriveState() == DriveState.BALANCING_ANGLE_LOCK) {
        this.drive.setDriveState(DriveState.BALANCING_ANGLE_LOCK);
      } else {
        this.drive.setDriveState(DriveState.BALANCING);
      }
    } else if (this.drive.getDriveState() == DriveState.HUMAN_AUTO_AIM) {
      this.io.setLimelightPipeline(1);
      this.io.setOdometryUsingLimelight(false);
      this.drive.setDriveState(DriveState.OUTPUT);
    }

    if (!this.driverIn.getBalanceButton()) {
      if (this.drive.getDriveState() == DriveState.BALANCING) {
        this.drive.setDriveState(DriveState.OUTPUT);
      } else if (this.drive.getDriveState() == DriveState.BALANCING_ANGLE_LOCK) {
        this.drive.setDriveState(DriveState.ANGLE_LOCK);
      }
    }

    if (this.driverIn.getDriverLeftStationButton()) {
      this.drive.setTargetingLeftStation(true);
    } else if (this.driverIn.getDriverRightStationButton()) {
      this.drive.setTargetingLeftStation(false);
    }

    if (Math.abs(rightX) > 0.05) {
      if (this.drive.getDriveState() == DriveState.ANGLE_LOCK) {
        this.drive.setDriveState(DriveState.OUTPUT);
      } else if (this.drive.getDriveState() == DriveState.BALANCING_ANGLE_LOCK) {
        this.drive.setDriveState(DriveState.BALANCING);
      }
    }

    // ARM

    if (this.arm.getArmState() == ArmState.HUMAN_LOADING_STATION) {
      this.io.setVisionAreaCutoff(RobotConstants.VISION_HUMAN_LOAD_AREA_CUTOFF);
    } else {
      this.io.setVisionAreaCutoff(RobotConstants.VISION_SCORING_AREA_CUTOFF);
    }

    boolean holdingOrIntaking =
        this.arm.getArmState() == ArmState.HOLDING
            || this.arm.getArmState() == ArmState.INTAKING_CUBE
            || this.arm.getArmState() == ArmState.INTAKING_CONE_TIPPED
            || this.arm.getArmState() == ArmState.INTAKING_CONE;

    if (this.driverIn.getArmScoringHighButton()) {
      if (this.intake.getRollerState() == RollerState.HOLDING_CONE) {
        this.arm.setArmWristState(ArmState.CONE_HIGH, WristState.SCORING_CONE_HIGH);
      } else {
        this.arm.setArmWristState(ArmState.CUBE_HIGH, WristState.SCORING_CUBE_HIGH);
      }

    } else if (this.driverIn.getArmScoringMiddleButton()) {
      if (this.intake.getRollerState() == RollerState.HOLDING_CONE) {
        if (this.intake.isConeTipped()) {
          this.arm.setArmWristState(
              ArmState.CONE_MEDIUM_TIPPED, WristState.SCORING_CONE_TIPPED_MEDIUM);
        } else {
          this.arm.setArmWristState(ArmState.CONE_MEDIUM, WristState.SCORING_CONE_MEDIUM);
        }

      } else {
        this.arm.setArmWristState(ArmState.CUBE_MEDIUM, WristState.SCORING_CUBE_MEDIUM);
      }
    } else if (this.driverIn.getArmScoringLowButton()) {
      this.arm.setArmWristState(ArmState.CONE_LOW, WristState.SCORING_LOW);
    } else if (this.driverIn.getArmHumanLoadingLevelButton()) {
      this.arm.setArmWristState(ArmState.HUMAN_LOADING_STATION, WristState.INTAKING_HUMAN_LOADING);
      this.intake.setRollerState(RollerState.INTAKING);
      this.intake.setExpectingTippedCone(false);
    } else if (this.driverIn.getIntakeConeButton() && holdingOrIntaking) {
      this.arm.setArmWristState(ArmState.INTAKING_CONE, WristState.INTAKING_CONE);
      this.intake.setExpectingTippedCone(false);
      this.intake.setExpectingCube(false);
    } else if (this.driverIn.getIntakeConeTippedButton() && holdingOrIntaking) {
      this.arm.setArmWristState(ArmState.INTAKING_CONE_TIPPED, WristState.INTAKING_CONE_TIPPED);
      this.intake.setExpectingCube(false);
      this.intake.setExpectingTippedCone(true);
    } else if (this.driverIn.getIntakeCubeButton() && holdingOrIntaking) {
      this.arm.setArmWristState(ArmState.INTAKING_CUBE, WristState.INTAKING_CUBE);
      this.intake.setExpectingCube(true);
      this.intake.setExpectingTippedCone(false);
    } else if (this.driverIn.getArmHoldingPositionButton()) {
      this.arm.setArmWristState(ArmState.HOLDING, WristState.HOLDING);
    }

    // INTAKE
    if (this.driverIn.getWristEncoderResetButton()) {
      this.io.resetWristEncoder();
    } else if ((this.driverIn.getIntakeConeButton()
            || this.driverIn.getIntakeConeTippedButton()
            || this.driverIn.getIntakeCubeButton())
        && this.intake.getRollerState() == RollerState.OFF) {
      this.intake.setRollerState(RollerState.INTAKING);
    } else if (this.driverIn.getDriverScoreButton()
        && (this.intake.getRollerState() == RollerState.HOLDING_CONE
            || this.intake.getRollerState() == RollerState.HOLDING_CUBE)) {

      switch (this.arm.getArmState()) {
        case CONE_HIGH -> {
          this.intake.setRollerState(RollerState.SCORING_HIGH_CONE);
          this.arm.setArmWristState(ArmState.CONE_HIGH_SCORING, WristState.SCORING_CONE_HIGH_DROP);
        }
        case CUBE_HIGH -> {
          this.intake.setRollerState(RollerState.SCORING_HIGH_CUBE);
        }
        case CUBE_MEDIUM -> {
          this.intake.setRollerState(RollerState.SCORING_MID_CUBE);
          this.arm.setArmWristState(ArmState.HOLDING, WristState.RETURNING_CUBE_MEDIUM);
        }
        case CONE_MEDIUM -> {
          this.intake.setRollerState(RollerState.SCORING_MID_CONE);
          this.arm.setArmWristState(ArmState.HOLDING, WristState.SCORING_CONE_MEDIUM);
        }
        case CONE_MEDIUM_TIPPED -> {
          this.intake.setRollerState(RollerState.SCORING_MID_CONE_TIPPED);
          this.arm.setArmWristState(
              ArmState.CONE_MEDIUM_TIPPED, WristState.SCORING_CONE_TIPPED_MEDIUM);
        }
        case CUBE_LOW -> {
          this.intake.setRollerState(RollerState.SCORING_LOW);
        }
        case CONE_LOW -> {
          this.intake.setRollerState(RollerState.SCORING_LOW);
        }
        case HOLDING -> {
          this.intake.setRollerState(RollerState.SCORING_LOW);
        }
        default -> {
          // not defined yet?
        }
      }
    }

    if (this.intake.getRollerState() == RollerState.HOLDING_CUBE && this.driverIn.getConeButton()) {
      this.intake.setRollerState(RollerState.HOLDING_CONE);
      this.intake.setExpectingCube(false);
    } else if (this.intake.getRollerState() == RollerState.HOLDING_CONE
        && this.driverIn.getCubeButton()) {
      this.intake.setRollerState(RollerState.HOLDING_CUBE);
      this.intake.setExpectingCube(true);
    }

    if (this.intake.getRollerState() == RollerState.OFF) {
      if (!List.of(
              ArmState.HOLDING,
              ArmState.MANUAL,
              ArmState.INTAKING_CONE,
              ArmState.HUMAN_LOADING_STATION,
              ArmState.CUBE_MEDIUM,
              ArmState.POST_MACONE,
              ArmState.POST_MACONE_TIPPED_MID,
              ArmState.CUBE_HIGH)
          .contains(this.arm.getArmState())) {

        if (this.arm.getArmState() == ArmState.CONE_HIGH_SCORING) {
          this.arm.setArmWristState(ArmState.POST_MACONE, this.arm.getWristState());
        } else if (this.arm.getArmState() == ArmState.CONE_MEDIUM_TIPPED) {
          this.arm.setArmWristState(ArmState.POST_MACONE_TIPPED_MID, this.arm.getWristState());
        } else {
          this.arm.setArmWristState(ArmState.HOLDING, this.arm.getWristState());
        }
      }
    }

    if ((this.intake.getRollerState() == RollerState.HOLDING_CONE
            || this.intake.getRollerState() == RollerState.HOLDING_CUBE)
        && (this.arm.getArmState() == ArmState.INTAKING_CONE
            || this.arm.getArmState() == ArmState.INTAKING_CONE_TIPPED
            || this.arm.getArmState() == ArmState.INTAKING_CUBE)) {
      this.arm.setArmWristState(ArmState.HOLDING, WristState.HOLDING);
    }

    // Manual mode override button is pressed by the operator.
    if (this.driverIn.getOperatorManualOnButton()) {
      this.arm.setArmWristState(ArmState.MANUAL, WristState.MANUAL);
    }

    leftX = this.xLimiter.calculate(leftX);
    leftY = this.yLimiter.calculate(leftY);
    rightX = this.turningLimiter.calculate(rightX);

    if (this.driverIn.getResetGyroButton()) {
      this.drive.resetGyro();
      this.drive.resetPose();
    }

    this.drive.setOutput(leftY, leftX, rightX);

    this.arm.setArmOutput(this.driverIn.getArmManualProximal(), this.driverIn.getArmManualDistal());

    this.arm.setWristOutput(this.driverIn.getWristManual());

    this.arm.setOperatorOffset(
        this.driverIn.getArmManualProximal(), this.driverIn.getArmManualDistal());

    // LEDS!
    if (this.intake.getRollerState() == RollerState.HOLDING_CONE) {
      this.ledstrips.setAllToState(LEDColourState.INTAKED_CONE);
    } else if (this.intake.getRollerState() == RollerState.HOLDING_CUBE) {
      this.ledstrips.setAllToState(LEDColourState.INTAKED_CUBE);
    } else if (this.driverIn.getConeButton()
        && this.intake.getRollerState() != RollerState.HOLDING_CONE
        && this.intake.getRollerState() != RollerState.HOLDING_CUBE) {
      this.ledstrips.setBlinkingCone();
      this.intake.setExpectingCube(false);
    } else if (this.driverIn.getCubeButton()
        && this.intake.getRollerState() != RollerState.HOLDING_CONE
        && this.intake.getRollerState() != RollerState.HOLDING_CUBE) {
      this.ledstrips.setBlinkingCube();
      this.intake.setExpectingCube(true);
    } else if (isScoring) {
      this.ledstrips.setAllToState(LEDColourState.OFF);
    }

    if (this.driverIn.getLeftBlinkerButton()) {
      this.ledstrips.setBlinkingLeft();
    } else if (this.driverIn.getRightBlinkerButton()) {
      this.ledstrips.setBlinkingRight();
    }

    this.drive.calculate();
    this.intake.calculate();
    this.arm.calculate();
  }

  @Override
  public void disable() {
    this.drive.disable();
    this.intake.disable();
    this.arm.disable();
  }
}
