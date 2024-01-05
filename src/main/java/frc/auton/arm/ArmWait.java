package frc.auton.arm;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;

public class ArmWait extends AutonCommand {

  public ArmWait() {
    super(RobotComponent.ARM);
  }

  @Override
  public boolean calculate() {
    return true;
  }

  @Override
  public void override() {
    // TODO Auto-generated method stub

  }

  @Override
  public void firstCycle() {
    // TODO Auto-generated method stub

  }
}
