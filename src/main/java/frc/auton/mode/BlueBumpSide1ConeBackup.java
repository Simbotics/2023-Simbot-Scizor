package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.arm.ArmWaitUntilPosition;
import frc.auton.arm.ArmWristSetState;
import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveWait;
import frc.auton.intake.IntakePickupCone;
import frc.auton.intake.IntakeScoreMidCone;
import frc.auton.intake.IntakeSetState;
import frc.auton.intake.IntakeWait;
import frc.auton.util.AutonWait;
import frc.robot.FieldConstants;
import frc.subsystems.Arm.ArmState;
import frc.subsystems.Arm.WristState;
import frc.subsystems.Intake.RollerState;

/**
 * @author Michael
 */
public class BlueBumpSide1ConeBackup implements AutonMode {

  @Override
  public void addToMode(AutonBuilder ab) {

    ab.addCommand(new DriveSetPosition((FieldConstants.BlueConeScoringLocations.Blue9.pos)));

    ab.addCommand(new IntakeSetState(RollerState.HOLDING_CONE));
    ab.addCommand(new AutonWait(500));

    ab.addCommand(new ArmWaitUntilPosition(ArmState.CONE_HIGH, WristState.SCORING_CONE_HIGH, 2000));
    ab.addCommand(
        new ArmWristSetState(
            ArmState.CONE_HIGH,
            WristState.SCORING_CONE_HIGH)); // instead of arm wait so we will continue to hold

    ab.addCommand(new AutonOverride(RobotComponent.INTAKE));

    ab.addCommand(new IntakeScoreMidCone(1000));

    ab.addCommand(new AutonOverride(RobotComponent.ARM));
    ab.addCommand(new ArmWaitUntilPosition(ArmState.HOLDING, WristState.HOLDING, 1500));
    ab.addCommand(
        new ArmWristSetState(
            ArmState.HOLDING,
            WristState.HOLDING)); // instead of arm wait so we will continue to hold

    ab.addCommand(new DriveToPoint(4.2, 0, 0.0, 0.0, 0.6, 0.2, 0.05, 5000));
    ab.addCommand(new DriveWait());

    ab.addCommand(new AutonOverride(RobotComponent.ARM));
    ab.addCommand(new ArmWaitUntilPosition(ArmState.INTAKING_CONE, WristState.INTAKING_CONE, 1500));
    ab.addCommand(
        new ArmWristSetState(
            ArmState.INTAKING_CONE,
            WristState.INTAKING_CONE)); // instead of arm wait so we will continue to hold

    ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
    ab.addCommand(new IntakePickupCone(3000));

    ab.addCommand(new DriveToPoint(4.75, 0, 0.0, 0.0, 0.3, 0.2, 0.08, 5000));
    ab.addCommand(new DriveWait());

    ab.addCommand(new IntakeWait());

    ab.addCommand(new AutonOverride(RobotComponent.ARM));
    ab.addCommand(new ArmWaitUntilPosition(ArmState.HOLDING, WristState.HOLDING, 1500));
    ab.addCommand(
        new ArmWristSetState(
            ArmState.HOLDING,
            WristState.HOLDING)); // instead of arm wait so we will continue to hold

    ab.addCommand(new DriveToPoint(1.0, 0, 180.0, 0.0, 0.3, 0.5, 0.05, 5000));
    ab.addCommand(new DriveWait());
  }
}
