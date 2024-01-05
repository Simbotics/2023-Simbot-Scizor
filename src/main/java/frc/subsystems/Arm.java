package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.util.SimPID;

public class Arm extends SubsystemBase {

  private static Arm instance;

  // Arm PIDs.
  private SimPID distalArmPID;
  private SimPID proximalArmPID;

  private double distalArmManual;
  private double proximalArmManual;

  private double wristManual;

  private double proximalOperatorOffset = 0;
  private double distalOperatorOffset = 0;

  // Wrist PID
  private SimPID wristPid;

  // Default arm state.
  public ArmState currentArmState = ArmState.MANUAL;
  private WristState currentWristState = WristState.MANUAL;

  /** Every state the arm can be in. */
  public enum ArmState {
    HOLDING,
    INTAKING_CONE, // Idle states.
    INTAKING_CONE_TIPPED,
    INTAKING_CUBE,
    CONE_LOW,
    CUBE_LOW, // Scoring low states.
    CONE_MEDIUM,
    CONE_MEDIUM_AUTO,
    CONE_MEDIUM_TIPPED,
    CUBE_MEDIUM, // Scoring medium states.
    CONE_HIGH,
    CONE_HIGH_SCORING,
    CUBE_HIGH, // Scoring high states/
    MANUAL, // MANUAL CONTROL
    HUMAN_LOADING_STATION,
    POST_MACONE,
    POST_MACONE_TIPPED_MID,
  }

  public enum WristState {
    HOLDING,
    INTAKING_CONE,
    INTAKING_CONE_TIPPED,
    INTAKING_CUBE,
    INTAKING_HUMAN_LOADING,
    SCORING_LOW,
    SCORING_CONE_MEDIUM,
    SCORING_CONE_MEDIUM_AUTO,
    SCORING_CONE_TIPPED_MEDIUM,
    SCORING_CUBE_MEDIUM,
    SCORING_CONE_HIGH,
    SCORING_CONE_HIGH_DROP,
    SCORING_CUBE_HIGH,
    RETURNING_CUBE_MEDIUM,
    RETURNING_CUBE_HIGH,
    MANUAL
  }

  /**
   * Gets the current state of the arm.
   *
   * @return The current state of the arm.
   */
  public ArmState getArmState() {
    return this.currentArmState;
  }

  public WristState getWristState() {
    return this.currentWristState;
  }

  /**
   * Sets the current state of the arm.
   *
   * @param armState The state to set the arm to.
   */
  public void setArmWristState(ArmState armState, WristState wristState) {
    this.currentArmState = armState;
    this.currentWristState = wristState;
  }

  public void setArmOutput(double proximal, double distal) {
    this.distalArmManual = distal;
    this.proximalArmManual = proximal;
  }

  public void setWristOutput(double wrist) {
    this.wristManual = wrist;
  }

  public void setOperatorOffset(double proximal, double distal) {
    this.distalOperatorOffset = distal * 11.0;
    this.proximalOperatorOffset = proximal * 5.0;
  }

  private void setArmsWithGravity(double proximal, double distal) {
    double distalOutput =
        distal
            + (RobotConstants.DISTAL_ARM_GRAVITY_OFFSET
                * Math.sin(Math.toRadians(this.io.getDistalArmAngle())));
    double proximalOutput =
        proximal
            + (RobotConstants.PROXIMAL_ARM_GRAVITY_OFFSET
                * Math.sin(Math.toRadians(this.io.getProximalArmAngle())));

    this.io.setDistalArm(distalOutput);
    this.io.setProximalArm(proximalOutput);
  }

  /**
   * Gets the current arm instance.
   *
   * @return The current arm instance; If there isn't one, create one.
   */
  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }

    return instance;
  }

  @Override
  public void firstCycle() {
    // Define the distal and proximal arm PIDs
    this.distalArmPID = new SimPID(RobotConstants.getDistalArmUpPID());
    this.proximalArmPID = new SimPID(RobotConstants.getProximalArmPID());

    this.proximalArmPID.setIRange(5);

    // define wrist PID
    this.wristPid = new SimPID(RobotConstants.getWristPID());

    this.wristPid.setIRange(5);
    this.wristPid.setMaxOutput(0.8);
    this.wristPid.setFinishedRange(1.5);
  }

  @Override
  public void calculate() {
    switch (this.currentArmState) {
      case HOLDING -> {
        // Set arms to holding angles.
        if (this.io.getDistalArmAngle() < 20
            && this.io.getProximalArmAngle() > 0) { // need to lift up distal
          this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_HOLDING_ANGLE + 25);
          this.proximalArmPID.setDesiredValue(5);
        } else if (this.io.getProximalArmAngle() > 0 && this.io.getDistalArmAngle() > 20) {
          this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);
          this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_HOLDING_ANGLE + 25);
        } else if (this.io.getProximalArmAngle() < 0) {
          this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);
          if (this.io.getWristAngle() > RobotConstants.WRIST_HOLDING_THRESHOLD) {

            this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_HOLDING_ANGLE);
          }
        }

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case INTAKING_CONE -> {
        // Set arms to intaking angles

        if (this.io.getProximalArmAngle() > 0) {
          this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_INTAKING_CONE_TIPPED_ANGLE);
        } else {
          this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_HOLDING_ANGLE);
        }

        this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_INTAKING_CONE_TIPPED_ANGLE);

        this.setArmsWithGravity(
            this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
            this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
      }

      case INTAKING_CONE_TIPPED -> {
        // Set arms to intaking angles.
        if (this.io.getProximalArmAngle() > 0) {
          this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_INTAKING_CONE_TIPPED_ANGLE);
        } else {
          this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_HOLDING_ANGLE);
        }
        this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_INTAKING_CONE_TIPPED_ANGLE);

        this.setArmsWithGravity(
            this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
            this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
      }

      case POST_MACONE -> {
        // Set arms to holding angles.

        this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_SCORING_HIGH_CONE_ANGLE - 7);

        this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case INTAKING_CUBE -> {
        // Set arms to intaking angles.
        this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_INTAKING_CUBE_ANGLE);
        this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_INTAKING_CUBE_ANGLE);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case CONE_LOW -> {
        // Set arms to cone low scoring angles.
        this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_SCORING_LOW_CONE_ANGLE);
        this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_SCORING_LOW_CONE_ANGLE);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case CUBE_LOW -> {
        // Set arms to cube low scoring angles.
        this.distalArmPID.setDesiredValue(RobotConstants.DISTAL_ARM_SCORING_LOW_CUBE_ANGLE);
        this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_SCORING_LOW_CUBE_ANGLE);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case CONE_MEDIUM -> {
        // Set arms to cone medium scoring angles.
        if (this.io.getDistalArmAngle() < RobotConstants.DISTAL_ARM_SCORING_THRESHOLD) {
          this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);
        } else {
          this.proximalArmPID.setDesiredValue(
              RobotConstants.PROXIMAL_ARM_SCORING_MEDIUM_CONE_ANGLE + this.proximalOperatorOffset);
        }
        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_SCORING_MEDIUM_CONE_ANGLE + this.distalOperatorOffset);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case CONE_MEDIUM_AUTO -> {
        // Set arms to cone medium scoring angles.
        if (this.io.getDistalArmAngle() < RobotConstants.DISTAL_ARM_SCORING_THRESHOLD) {
          this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);
        } else {
          this.proximalArmPID.setDesiredValue(
              RobotConstants.PROXIMAL_ARM_SCORING_MEDIUM_CONE_ANGLE + this.proximalOperatorOffset);
        }
        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_SCORING_MEDIUM_CONE_ANGLE + this.distalOperatorOffset - 10);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case CONE_MEDIUM_TIPPED -> {
        // Set arms to cone medium scoring angles.
        if (this.io.getDistalArmAngle() < RobotConstants.DISTAL_ARM_SCORING_THRESHOLD) {
          this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);
        } else {
          this.proximalArmPID.setDesiredValue(
              RobotConstants.PROXIMAL_ARM_SCORING_MEDIUM_CONE_TIPPED_ANGLE
                  + this.proximalOperatorOffset);
        }
        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_SCORING_MEDIUM_CONE_TIPPED_ANGLE + this.distalOperatorOffset);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case POST_MACONE_TIPPED_MID -> {
        // Set arms to cone medium scoring angles.

        this.proximalArmPID.setDesiredValue(
            RobotConstants.PROXIMAL_ARM_SCORING_MEDIUM_CONE_TIPPED_ANGLE
                + this.proximalOperatorOffset);

        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_SCORING_MEDIUM_CONE_TIPPED_ANGLE
                + this.distalOperatorOffset
                + 10);

        this.setArmsWithGravity(
            this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
            this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
      }

      case CUBE_MEDIUM -> {
        // Set arms to cube medium scoring angles.
        if (this.io.getProximalArmAngle() > -9) {
          this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);
        }

        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_SCORING_MEDIUM_CUBE_ANGLE + this.distalOperatorOffset);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case CONE_HIGH -> {
        // Set arms to cone high scoring angles.
        if (this.io.getDistalArmAngle() < RobotConstants.DISTAL_ARM_SCORING_THRESHOLD - 45) {
          this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);
        } else {
          this.proximalArmPID.setDesiredValue(
              RobotConstants.PROXIMAL_ARM_SCORING_HIGH_CONE_ANGLE + this.proximalOperatorOffset);
        }
        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_SCORING_HIGH_CONE_ANGLE + this.distalOperatorOffset);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }

      case CONE_HIGH_SCORING -> {
        // Set arms to cone high scoring angles.

        this.proximalArmPID.setDesiredValue(
            RobotConstants.PROXIMAL_ARM_SCORING_HIGH_CONE_ANGLE + this.proximalOperatorOffset - 7);

        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_SCORING_HIGH_CONE_ANGLE + this.distalOperatorOffset - 15);

        this.setArmsWithGravity(
            this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
            this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
      }

      case CUBE_HIGH -> {
        // Sets arms to cube high scoring angles.
        if (this.io.getProximalArmAngle() > -9) {
          this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);
        }

        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_SCORING_HIGH_CUBE_ANGLE + this.distalOperatorOffset);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }
      case MANUAL -> {
        this.setArmsWithGravity(this.proximalArmManual * -0.3, this.distalArmManual * 0.3);
        this.io.setDistalArmRampRate(0.0);
        this.io.setProximalArmRampRate(0.0);
      }
      case HUMAN_LOADING_STATION -> {
        // Sets arms to cube high scoring angles.

        this.proximalArmPID.setDesiredValue(RobotConstants.PROXIMAL_ARM_HOLDING_ANGLE);

        this.distalArmPID.setDesiredValue(
            RobotConstants.DISTAL_ARM_HUMAN_LOADING_STATION_ANGLE + this.distalOperatorOffset);

        if (this.io.getProximalArmAngle() <= -9 && this.proximalArmPID.getDesiredVal() <= -9) {
          this.setArmsWithGravity(-0.06, this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        } else {
          this.setArmsWithGravity(
              this.proximalArmPID.calcPID(this.io.getProximalArmAngle()),
              this.distalArmPID.calcPID(this.io.getDistalArmAngle()));
        }
      }
      default -> {
        this.io.setDistalArm(0);
        this.io.setProximalArm(0);
      }
    }

    switch (this.currentWristState) {
      case HOLDING -> {
        if (this.io.getDistalArmAngle() < 5 && this.io.getProximalArmAngle() > 0) {
          this.wristPid.setDesiredValue(RobotConstants.INTAKING_CONE_ANGLE);
        } else {
          this.wristPid.setDesiredValue(RobotConstants.HOLDING_ANGLE);
        }
      }
      case INTAKING_CONE -> {
        this.wristPid.setDesiredValue(RobotConstants.INTAKING_CONE_TIPPED_ANGLE);
      }
      case INTAKING_CONE_TIPPED -> {
        this.wristPid.setDesiredValue(RobotConstants.INTAKING_CONE_TIPPED_ANGLE);
      }
      case INTAKING_CUBE -> {
        if (this.io.getDistalArmAngle() < RobotConstants.DISTAL_ARM_INTAKING_CUBE_THRESHOLD) {
          this.wristPid.setDesiredValue(RobotConstants.HOLDING_ANGLE);
        } else {
          this.wristPid.setDesiredValue(RobotConstants.INTAKING_CUBE_ANGLE);
        }
      }
      case INTAKING_HUMAN_LOADING -> {
        this.wristPid.setDesiredValue(RobotConstants.INTAKING_HUMAN_LOADING_ANGLE);
      }
      case SCORING_LOW -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_LOW_ANGLE);
      }
      case SCORING_CONE_MEDIUM -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_CONE_MID_ANGLE);
      }

      case SCORING_CONE_MEDIUM_AUTO -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_CONE_MID_ANGLE - 15);
      }

      case SCORING_CONE_TIPPED_MEDIUM -> {
        if (this.io.getDistalArmAngle() > 25) {
          this.wristPid.setDesiredValue(RobotConstants.SCORING_CONE_MID_TIPPED_ANGLE);
        } else {
          this.wristPid.setDesiredValue(RobotConstants.HOLDING_ANGLE);
        }
      }
      case SCORING_CUBE_MEDIUM -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_CUBE_MID_ANGLE);
      }
      case SCORING_CONE_HIGH -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_CONE_HIGH_ANGLE);
      }

      case SCORING_CONE_HIGH_DROP -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_CONE_HIGH_ANGLE - 20);
      }
      case SCORING_CUBE_HIGH -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_CUBE_HIGH_ANGLE);
      }
      case RETURNING_CUBE_MEDIUM -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_CUBE_MID_ANGLE + 20);
      }
      case RETURNING_CUBE_HIGH -> {
        this.wristPid.setDesiredValue(RobotConstants.SCORING_CUBE_HIGH_ANGLE + 5);
      }
      case MANUAL -> {
        this.io.setWristMotor(wristManual);
      }
    }

    // Always set the distal arm to target the desired angle.
    if (this.currentArmState != ArmState.MANUAL) {
      if (Math.abs(this.distalArmPID.getDesiredVal() - this.io.getDistalArmAngle()) > 20) {
        this.io.setDistalArmRampRate(0.5);
      } else {
        this.io.setDistalArmRampRate(0.1);
      }

      if (Math.abs(this.proximalArmPID.getDesiredVal() - this.io.getProximalArmAngle()) > 15) {
        this.io.setProximalArmRampRate(0.8);
      } else {
        this.io.setProximalArmRampRate(0.1);
      }

      if (this.distalArmPID.getDesiredVal() - this.io.getDistalArmAngle() > 0) { // we are going up
        this.distalArmPID.setConstants(RobotConstants.getDistalArmUpPID());
      } else {
        this.distalArmPID.setConstants(RobotConstants.getDistalArmDownPID());
      }
    }

    if (this.currentWristState != WristState.MANUAL) {
      this.io.setWristMotor(this.wristPid.calcPID(this.io.getWristAngle()));
    }

    SmartDashboard.putString("ARM STATE", this.currentArmState.toString());
    SmartDashboard.putNumber("DistalManual", distalOperatorOffset);
    SmartDashboard.putNumber("Distal Target", this.distalArmPID.getDesiredVal());
    SmartDashboard.putString("WRIST STATE", this.currentWristState.toString());
  }

  public boolean isArmInPosition() {
    return (Math.abs((this.proximalArmPID.getDesiredVal() - this.io.getProximalArmAngle())) < 2.0)
        && Math.abs((this.distalArmPID.getDesiredVal() - this.io.getDistalArmAngle())) < 2.0
        && Math.abs((this.wristPid.getDesiredVal() - this.io.getWristAngle())) < 5.0;
  }

  @Override
  public void disable() {
    this.io.setDistalArm(0);
    this.io.setProximalArm(0);
  }
}
