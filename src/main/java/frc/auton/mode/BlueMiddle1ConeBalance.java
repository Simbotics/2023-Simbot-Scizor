package frc.auton.mode;

import frc.auton.AutonGroups;
import frc.auton.drive.DriveSetPosition;
import frc.auton.intake.IntakeSetState;
import frc.auton.util.AutonWait;
import frc.auton.util.ReversibleType;
import frc.robot.FieldConstants;
import frc.subsystems.Intake.RollerState;

/**
 * @author Michael
 */
public class BlueMiddle1ConeBalance implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {
    // (really, do nothing, it's ok)
    // ab.addCommand(new DriveSetPosition(0, 0, 0));
    ab.addCommand(new DriveSetPosition((FieldConstants.BlueConeScoringLocations.Blue6.pos)));

    ab.addCommand(new IntakeSetState(RollerState.HOLDING_CONE));
    ab.addCommand(new AutonWait(150));

    AutonGroups.PreloadCommands.SCORE_HIGH_CONE.addToAutonBuilder(ab);

    AutonGroups.PreloadAndBalanceCommands.DRIVE_AND_BALANCE_MID
        .reverse(ReversibleType.ALLIANCE)
        .addToAutonBuilder(ab);
  }
}
