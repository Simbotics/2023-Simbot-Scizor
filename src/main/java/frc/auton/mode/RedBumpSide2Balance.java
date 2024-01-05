package frc.auton.mode;

import frc.auton.AutonGroups;
import frc.auton.drive.DriveSetPosition;
import frc.auton.intake.IntakeSetState;
import frc.auton.util.AutonWait;
import frc.auton.util.ReversibleType;
import frc.robot.FieldConstants;
import frc.subsystems.Intake.RollerState;

/**
 * @author Julian
 */

// Cone, Cube, Cone
public class RedBumpSide2Balance implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {

    // start near BUMP player side
    ab.addCommand(new DriveSetPosition((FieldConstants.RedConeScoringLocations.Red1.pos)));

    // properly holds cone
    ab.addCommand(new IntakeSetState(RollerState.HOLDING_CONE));
    ab.addCommand(new AutonWait(150));

    AutonGroups.PreloadCommands.SCORE_MID_CONE.reverse(ReversibleType.BUMP).addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_FIRST_CONE_TO_FIRST_PIECE
        .reverse(ReversibleType.BUMP)
        .reverse(ReversibleType.SLOW)
        .addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.PICK_UP_CUBE.reverse(ReversibleType.BUMP).addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_FIRST_CUBE_SCORE_HIGH
        .reverse(ReversibleType.BUMP)
        .reverse(ReversibleType.SLOW)
        .addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_CUBE_NODE_TO_GET_SECOND_CUBE
        .reverse(ReversibleType.BUMP)
        .reverse(ReversibleType.SLOW)
        .addToAutonBuilder(ab);
    AutonGroups.TwoAndBalanceCommands.DRIVE_AND_BALANCE
        .reverse(ReversibleType.BUMP)
        .addToAutonBuilder(ab);
  }
}
