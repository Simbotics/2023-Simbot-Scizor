package frc.auton.util;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;

public class TestPrint extends AutonCommand {

  private String toPrint;

  public TestPrint(String s) {
    super(RobotComponent.UTIL);
    this.toPrint = s;
  }

  @Override
  public void firstCycle() {
    // TODO Auto-generated method stub

  }

  @Override
  public boolean calculate() {
    System.out.println(this.toPrint);
    return true;
  }

  @Override
  public void override() {
    // TODO Auto-generated method stub

  }
}
