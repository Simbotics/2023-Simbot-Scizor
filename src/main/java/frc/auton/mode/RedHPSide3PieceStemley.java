package frc.auton.mode;

import frc.auton.AutonGroups;
import frc.auton.drive.DriveSetPosition;
import frc.auton.intake.IntakeSetState;
import frc.auton.util.AutonWait;
import frc.robot.FieldConstants;
import frc.subsystems.Intake.RollerState;

/**
 * @author Julian
 */

// Cone, Cube, Cone
public class RedHPSide3PieceStemley implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {

    // start near human player side
    ab.addCommand(new DriveSetPosition((FieldConstants.RedConeScoringLocations.Red9.pos)));
    ab.addCommand(
        new DriveSetPosition(
            false,
            FieldConstants.RedConeScoringLocations.Red9.pos.getX() - 1.22,
            FieldConstants.RedConeScoringLocations.Red9.pos.getY(),
            FieldConstants.RedConeScoringLocations.Red9.pos.getRotation().getDegrees()));
    ab.addCommand(new IntakeSetState(RollerState.HOLDING_CONE));

    ab.addCommand(new AutonWait(50));

    AutonGroups.PreloadCommands.SCORE_LOW.addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_FIRST_CONE_TO_FIRST_PIECE.addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.PICK_UP_CUBE.addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_FIRST_CUBE_SCORE_HIGH.addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_CUBE_NODE_TO_GET_SECOND_CUBE.addToAutonBuilder(ab);
    AutonGroups.ThreePieceCommands.DRIVE_FROM_SECOND_PIECE_TO_INSIDE_CONE_NODE.addToAutonBuilder(
        ab);
  }
}
