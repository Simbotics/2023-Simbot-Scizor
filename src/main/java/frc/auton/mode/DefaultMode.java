package frc.auton.mode;

import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveWait;
import frc.auton.util.RunFirstCycle;
import frc.robot.FieldConstants;

/**
 * @author Michael
 */
public class DefaultMode implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {
    // (really, do nothing, it's ok)
    ab.addCommand(new DriveSetPosition((FieldConstants.RedConeScoringLocations.Red9.pos)));
    ab.addCommand(new RunFirstCycle());

    ab.addCommand(new DriveWait());
  }
}
