package frc.auton.intake;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Intake;
import frc.subsystems.Intake.RollerState;

public class IntakeScoreHighCube extends AutonCommand {

  private Intake intake;
  private IO io;

  public IntakeScoreHighCube(long timeout) {
    super(RobotComponent.INTAKE, timeout);
    this.io = IO.getInstance();
    this.intake = Intake.getInstance();
  }

  @Override
  public void firstCycle() {
    intake.setRollerState(RollerState.SCORING_HIGH_CUBE_AUTO);
  }

  @Override
  public boolean calculate() {
    intake.calculate();
    return this.intake.getRollerState() == RollerState.OFF;
  }

  @Override
  public void override() {
    this.io.setRollerMotor(0.0);
  }
}
