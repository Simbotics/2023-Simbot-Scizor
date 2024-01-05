package frc.auton.mode;

import frc.auton.drive.DriveBalance;
import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveWait;
import frc.robot.FieldConstants;

/**
 * @author Michael
 */
public class DriveBalanceTest implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {
    ab.addCommand(new DriveSetPosition((FieldConstants.RedConeScoringLocations.Red4.pos)));

    ab.addCommand(new DriveToPoint(-1.0, 0, 0, 0.4, 0.4, 0.2, 0.1, 5000));
    ab.addCommand(new DriveBalance(1, 0, 10000));
    ab.addCommand(new DriveWait());
  }
}
