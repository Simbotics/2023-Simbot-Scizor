package frc.auton.util;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.subsystems.Drive;
import frc.subsystems.Intake;

public class RunFirstCycle extends AutonCommand {
  private Drive drive;
  private Intake intake;

  public RunFirstCycle() {
    super(RobotComponent.UTIL);
    this.drive = Drive.getInstance();
    this.intake = Intake.getInstance();
  }

  @Override
  public void firstCycle() {
    this.drive.firstCycle();
    this.intake.firstCycle();
  }

  @Override
  public boolean calculate() {
    return true;
  }

  @Override
  public void override() {}
}
