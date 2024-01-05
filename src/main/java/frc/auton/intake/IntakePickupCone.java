package frc.auton.intake;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Intake;
import frc.subsystems.Intake.RollerState;

public class IntakePickupCone extends AutonCommand {

  private Intake intake;
  private IO io;

  public IntakePickupCone(long timeout) {
    super(RobotComponent.INTAKE, timeout);
    this.io = IO.getInstance();
    this.intake = Intake.getInstance();
  }

  @Override
  public void firstCycle() {
    intake.setExpectingCube(false);
    intake.setExpectingTippedCone(false);
    intake.setRollerState(RollerState.INTAKING);
  }

  @Override
  public boolean calculate() {
    intake.calculate();
    if (this.intake.getRollerState() == RollerState.HOLDING_CONE) {
      intake.calculate();
      this.io.setRollerMotor(0.1);
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void override() {
    this.io.setRollerMotor(0);
  }
}
