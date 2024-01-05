package frc.auton.mode;

import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveWait;
import frc.robot.FieldConstants;

/**
 * @author Michael
 */
public class DriveOrientationTest implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {
    ab.addCommand(new DriveSetPosition((FieldConstants.BlueConeScoringLocations.Blue1.pos)));

    ab.addCommand(new DriveToPoint(1.0, 0.75, 180.0 - 155.0, 0.0, 0.2, 0.2, 0.05, 5000));
    ab.addCommand(new DriveWait());
  }
}
