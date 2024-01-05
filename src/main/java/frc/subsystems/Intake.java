package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class Intake extends SubsystemBase {

  private int scoringCycles = 0;

  private int currentCycles = 0;

  private boolean expectingCube = true;
  private boolean tippedCone = false;

  private static Intake instance;

  private RollerState currentRollerState = RollerState.OFF;

  public enum RollerState {
    OFF,
    INTAKING,
    INTAKING_AUTO,
    SCORING_LOW,
    SCORING_LOW_AUTO,
    SCORING_MID_CONE,
    SCORING_MID_CONE_TIPPED,
    SCORING_HIGH_CONE,
    SCORING_MID_CUBE,
    SCORING_MID_CUBE_AUTO,
    SCORING_HIGH_CUBE,
    SCORING_HIGH_CUBE_AUTO,
    SCORING_HIGH_CONE_AUTO,
    HOLDING_CUBE,
    HOLDING_CONE
  }

  private Intake() {
    super();
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  public void setRollerState(RollerState currentRollerState) {
    this.currentRollerState = currentRollerState;
  }

  public RollerState getRollerState() {
    return this.currentRollerState;
  }

  public void setExpectingCube(boolean areWeIntakingCubes) {
    this.expectingCube = areWeIntakingCubes;
  }

  public void setExpectingTippedCone(boolean isConeTipped) {
    this.tippedCone = isConeTipped;
  }

  public boolean isConeTipped() {
    return this.tippedCone;
  }

  @Override
  public void firstCycle() { // TODO add roller first cycle
  }

  @Override
  public void calculate() {
    double current = this.io.getRollerCurrent();

    switch (this.currentRollerState) {
      case OFF -> {
        scoringCycles = 0;
        this.currentCycles = 0;
        this.io.setRollerMotor(0.0);
      }
      case INTAKING -> {
        scoringCycles = 0;
        SmartDashboard.putNumber("ROLLER AMPS", current);

        double angleThreshold;

        if (this.io.getIsAuto()) {
          angleThreshold = 15.0;
        } else {
          angleThreshold = 10.0;
        }

        if (this.io.getDistalArmAngle() > angleThreshold
            || this.io.getProximalArmAngle() > angleThreshold) {
          this.io.setRollerMotor(0.95);
          if (current > RobotConstants.ROLLER_AMPS) {
            this.currentCycles++;

            if (this.expectingCube) {
              if (this.currentCycles > 12) {
                this.setRollerState(RollerState.HOLDING_CUBE);
              }
            } else {
              if (this.currentCycles > 15) {
                this.setRollerState(RollerState.HOLDING_CONE);
              }
            }

          } else {
            this.currentCycles = 0;
          }
        } else {
          if (this.io.getDistalArmAngle() > 5 || this.io.getProximalArmAngle() > 5) {

            this.io.setRollerMotor(0.95);
          } else {
            this.io.setRollerMotor(0.0);
          }

          this.currentCycles = 0;
        }
      }
      case INTAKING_AUTO -> {
        scoringCycles = 0;
        this.currentCycles = 0;

        if (this.io.getDistalArmAngle() > 10 || this.io.getProximalArmAngle() > 10) {
          this.io.setRollerMotor(0.8);

        } else {
          this.io.setRollerMotor(0.0);
        }
      }

      case SCORING_MID_CONE -> {
        if (scoringCycles < 25) {
          scoringCycles++;
          this.io.setRollerMotor(-0.1);
        } else {
          scoringCycles = 0;
          this.io.setRollerMotor(0.0);
          this.setRollerState(RollerState.OFF);
        }
      }

      case SCORING_MID_CONE_TIPPED -> {
        if (scoringCycles < 25) {
          scoringCycles++;
          this.io.setRollerMotor(-0.8);
        } else {
          scoringCycles = 0;
          this.io.setRollerMotor(0.0);
          this.setRollerState(RollerState.OFF);
        }
      }

      case SCORING_MID_CUBE -> {
        if (scoringCycles < 25) {
          scoringCycles++;
          this.io.setRollerMotor(-0.25);
        } else {
          scoringCycles = 0;
          this.io.setRollerMotor(0.0);
          this.setRollerState(RollerState.OFF);
        }
      }

      case SCORING_MID_CUBE_AUTO -> {
        if (scoringCycles < 15) {
          scoringCycles++;
          this.io.setRollerMotor(-0.2);
        } else {
          scoringCycles = 0;
          this.io.setRollerMotor(0.0);
          this.setRollerState(RollerState.OFF);
        }
      }

      case SCORING_HIGH_CONE -> {
        if (scoringCycles < 65) {
          scoringCycles++;
          this.io.setRollerMotor(-0.15);
        } else {
          scoringCycles = 0;
          this.io.setRollerMotor(0.0);
          this.setRollerState(RollerState.OFF);
        }
      }

      case SCORING_HIGH_CONE_AUTO -> {
        if (scoringCycles < 35) {
          scoringCycles++;
          this.io.setRollerMotor(-0.15);
        } else {
          scoringCycles = 0;
          this.io.setRollerMotor(0.0);
          this.setRollerState(RollerState.OFF);
        }
      }

      case SCORING_HIGH_CUBE -> {
        if (scoringCycles < 25) {
          scoringCycles++;
          this.io.setRollerMotor(-0.55);
        } else {
          scoringCycles = 0;
          this.io.setRollerMotor(0.0);
          this.setRollerState(RollerState.OFF);
        }
      }

      case SCORING_HIGH_CUBE_AUTO -> {
        if (scoringCycles < 25) {
          scoringCycles++;
          this.io.setRollerMotor(-0.6);
        } else {
          scoringCycles = 0;
          this.io.setRollerMotor(0.0);
          this.setRollerState(RollerState.OFF);
        }
      }

      case SCORING_LOW -> {
        if (this.expectingCube) {
          if (scoringCycles < 50) {
            scoringCycles++;
            this.io.setRollerMotor(-0.3);
          } else {
            scoringCycles = 0;
            this.io.setRollerMotor(0.0);
            this.setRollerState(RollerState.OFF);
          }
        } else {
          if (scoringCycles < 50) {
            scoringCycles++;
            this.io.setRollerMotor(-0.6);
          } else {
            scoringCycles = 0;
            this.io.setRollerMotor(0.0);
            this.setRollerState(RollerState.OFF);
          }
        }
      }

      case SCORING_LOW_AUTO -> {
        if (this.expectingCube) {
          if (scoringCycles < 50) {
            scoringCycles++;
            this.io.setRollerMotor(-0.3);
          } else {
            scoringCycles = 0;
            this.io.setRollerMotor(0.0);
            this.setRollerState(RollerState.OFF);
          }
        } else {
          if (scoringCycles < 50) {
            scoringCycles++;
            this.io.setRollerMotor(-1.0);
          } else {
            scoringCycles = 0;
            this.io.setRollerMotor(0.0);
            this.setRollerState(RollerState.OFF);
          }
        }
      }

      case HOLDING_CUBE -> {
        scoringCycles = 0;
        this.currentCycles = 0;
        this.io.setRollerMotor(0.15);
      }
      case HOLDING_CONE -> {
        scoringCycles = 0;
        this.currentCycles = 0;
        this.io.setRollerMotor(0.18);
      }
    }

    SmartDashboard.putString("ROLLER STATE", this.currentRollerState.toString());
  }

  @Override
  public void disable() {}
}
