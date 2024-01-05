package frc.auton.mode;

import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveWait;

/**
 * @author Michael
 */
public class Drive2MetersInX implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {
    ab.addCommand(new DriveSetPosition(0, 0, 0));
    ab.addCommand(new DriveToPoint(2, 0, 0, 0, 1, 1, 0.01, 10000000));
    ab.addCommand(new DriveWait());
  }
}
