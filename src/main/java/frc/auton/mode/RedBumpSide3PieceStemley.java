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
public class RedBumpSide3PieceStemley implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {

    // start near human player side
    ab.addCommand(new DriveSetPosition((FieldConstants.RedConeScoringLocations.Red1.pos)));
    ab.addCommand(
        new DriveSetPosition(
            false,
            FieldConstants.RedConeScoringLocations.Red1.pos.getX() - 2.53,
            FieldConstants.RedConeScoringLocations.Red1.pos.getY(),
            FieldConstants.RedConeScoringLocations.Red9.pos.getRotation().getDegrees()));
    ab.addCommand(new IntakeSetState(RollerState.HOLDING_CONE));

    ab.addCommand(new AutonWait(50));

    AutonGroups.PreloadCommands.YEET_LOW.addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_FIRST_CONE_TO_FIRST_PIECE_BUMP
        .reverse(ReversibleType.BUMP)
        .reverse(ReversibleType.SLOW)
        .addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.PICK_UP_CUBE
        .reverse(ReversibleType.BUMP)
        .reverse(ReversibleType.SLOW)
        .addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_FIRST_CUBE_SCORE_HIGH
        .reverse(ReversibleType.BUMP)
        .reverse(ReversibleType.SLOW)
        .addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_CUBE_NODE_TO_GET_SECOND_CUBE
        .reverse(ReversibleType.BUMP)
        .reverse(ReversibleType.SLOW)
        .addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_SECOND_PIECE_TO_INSIDE_CONE_NODE
        .reverse(ReversibleType.BUMP)
        .reverse(ReversibleType.SLOW)
        .addToAutonBuilder(ab);
  }
}
