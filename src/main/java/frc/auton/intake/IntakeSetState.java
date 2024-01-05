package frc.auton.intake;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Intake;

public class IntakeSetState extends AutonCommand {

  private Intake intake;
  private Intake.RollerState intakeRollerState;

  private IO io;

  public IntakeSetState(Intake.RollerState intakeRollerState) {
    super(RobotComponent.INTAKE);
    this.intake = Intake.getInstance();
    this.io = IO.getInstance();
    this.intakeRollerState = intakeRollerState;
  }

  @Override
  public void firstCycle() {
    intake.setRollerState(this.intakeRollerState);
  }

  @Override
  public boolean calculate() {
    intake.calculate();
    return false;
  }

  @Override
  public void override() {
    this.io.setRollerMotor(0.0);
  }
}
