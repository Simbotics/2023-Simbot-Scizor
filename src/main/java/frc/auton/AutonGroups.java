package frc.auton;

import frc.auton.arm.*;
import frc.auton.drive.*;
import frc.auton.intake.*;
import frc.auton.util.AutonSetPipeline;
import frc.auton.util.AutonSetPipeline.Pipeline;
import frc.auton.util.AutonWait;
import frc.subsystems.Arm.ArmState;
import frc.subsystems.Arm.WristState;
import frc.subsystems.Intake.RollerState;

public class AutonGroups {

  public static final class PreloadCommands {

    public static final AutonCommandGroup SCORE_LOW =
        new AutonCommandGroup(
            // moves arm to low

            new ArmWristSetState(ArmState.CONE_MEDIUM, WristState.SCORING_CONE_HIGH),
            new AutonWait(250),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeScoreLowCone(500),
            new AutonWait(50),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(ArmState.INTAKING_CUBE, WristState.INTAKING_CUBE));

    public static final AutonCommandGroup YEET_LOW =
        new AutonCommandGroup(
            // moves arm to low

            new ArmWristSetState(ArmState.CONE_HIGH, WristState.SCORING_CONE_HIGH),
            new AutonWait(350),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeYeetCone(500),
            new AutonWait(250),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(ArmState.INTAKING_CUBE, WristState.INTAKING_CUBE));

    public static final AutonCommandGroup SCORE_MID_CONE =
        new AutonCommandGroup(
            // moves arm to medium cone
            new ArmWaitUntilPosition(ArmState.CONE_MEDIUM, WristState.SCORING_CONE_MEDIUM, 900),
            new ArmWaitUntilPosition(
                ArmState.CONE_MEDIUM_AUTO, WristState.SCORING_CONE_MEDIUM, 300),
            new ArmWait(),
            // scores medium cone
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.OFF),
            new ArmWaitUntilPosition(ArmState.HOLDING, WristState.SCORING_CONE_MEDIUM, 1300),
            new AutonWait(100));

    public static final AutonCommandGroup SCORE_HIGH_CONE =
        new AutonCommandGroup(
            new ArmWristSetState(ArmState.CONE_HIGH, WristState.SCORING_CONE_HIGH),
            new AutonWait(1300),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(
                ArmState.CONE_HIGH_SCORING,
                WristState
                    .SCORING_CONE_HIGH_DROP), // instead of arm wait so we will continue to hold
            new AutonWait(200),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeScoreHighCone(1500),
            new IntakeWait(),
            new AutonOverride(RobotComponent.ARM),
            new ArmWaitUntilPosition(ArmState.POST_MACONE, WristState.HOLDING, 400),
            new ArmWristSetState(
                ArmState.HOLDING, WristState.HOLDING) // instead of arm wait so we will cont
            );
  }

  public static final class ThreePieceCommands {

    public static final AutonCommandGroup DRIVE_FROM_FIRST_CONE_TO_FIRST_PIECE =
        new AutonCommandGroup(
            new AutonSetPipeline(Pipeline.CUBE_TRACKING),
            new DriveToPoint(-2.0, 0, 0.0, 0.7, 0.8, 0.5, 0.3, 3000),
            new DriveToPoint(-4.05, 0.35, 170.0, 0.3, 0.7, 0.6, 0.07, 3000));

    public static final AutonCommandGroup DRIVE_FROM_FIRST_CONE_TO_FIRST_PIECE_BUMP =
        new AutonCommandGroup(
            new AutonSetPipeline(Pipeline.CUBE_TRACKING),
            new DriveToPoint(-4.05, 0.35, 170.0, 0.3, 0.7, 0.6, 0.07, 3000));

    public static final AutonCommandGroup PICK_UP_CUBE =
        new AutonCommandGroup(
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.INTAKING_AUTO),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(
                ArmState.INTAKING_CUBE,
                WristState.INTAKING_CUBE), // instead of arm wait so we will continue to hold
            new DriveToCube(0.3, 170.0, 0.7, true, 750),
            new DriveWait(),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.HOLDING_CUBE),
            new AutonSetPipeline(Pipeline.APRIL_TAG),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(
                ArmState.HOLDING,
                WristState.HOLDING) // instead of arm wait so we will continue to hold
            );

    public static final AutonCommandGroup DRIVE_FROM_FIRST_CUBE_SCORE_HIGH =
        new AutonCommandGroup(

            // Driving to score cube
            new AutonSetPipeline(Pipeline.APRIL_TAG),
            new DriveToPoint(-2.7, 0.65, 0.0, 0.4, 0.6, 0.3, 0.1, 5000),
            new DriveToPoint(-0.2, 0.65, 0.0, 0.0, 0.45, 0.3, 0.1, 2000, true, true),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(
                ArmState.HOLDING,
                WristState.HOLDING), // instead of arm wait so we will continue to hold
            new DriveWait(),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeScoreLowCone(500),
            new IntakeWait());

    public static final AutonCommandGroup DRIVE_FROM_CUBE_NODE_TO_SECOND_PIECE =
        new AutonCommandGroup(
            new AutonOverride(RobotComponent.DRIVE),
            new AutonSetPipeline(Pipeline.CUBE_TRACKING),
            new DriveToPoint(-2.2, 0.35, 0.0, 0.65, 0.9, 0.2, 0.1, 5000));
    //  new DriveToPoint(-2.5, 0.5, 141.0, 0.7, 0.8, 0.5, 0.3, 5000));

    public static final AutonCommandGroup PICK_UP_SECOND_CUBE =
        new AutonCommandGroup(
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.INTAKING_AUTO),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(ArmState.INTAKING_CUBE, WristState.INTAKING_CUBE));

    public static final AutonCommandGroup DRIVE_FROM_CUBE_NODE_TO_GET_SECOND_CUBE =
        new AutonCommandGroup(
            DRIVE_FROM_CUBE_NODE_TO_SECOND_PIECE,
            PICK_UP_SECOND_CUBE,
            new DriveToPoint(-3.85, 1.4, 148.0, 0.25, 0.65, 0.5, 0.6, 1800),
            new DriveToCube(0.3, 148.0, 0.3, true, 850),
            new DriveWait(),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.HOLDING_CUBE),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(ArmState.HOLDING, WristState.HOLDING));

    public static final AutonCommandGroup DRIVE_FROM_SECOND_PIECE_TO_INSIDE_CONE_NODE =
        new AutonCommandGroup(
            new DriveToPoint(-1.8, 0.4, 0.0, 0.4, 0.6, 0.4, 0.1, 5000),
            new AutonSetPipeline(Pipeline.APRIL_TAG),
            new DriveToPoint(-0.24, 0.55, 0.0, 0.0, 0.4, 0.4, 0.12, 2000, true, true),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(
                ArmState.CUBE_HIGH,
                WristState.SCORING_CUBE_HIGH), // instead of arm wait so we will continue to hold
            new DriveWait(),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeScoreHighCube(2000),
            new IntakeWait(),
            new DriveToPoint(-0.3, 0.55, 0.0, 0.0, 0.4, 0.2, 0.05, 2500, true, true),
            new DriveWait());
  }

  public static final class TwoAndBalanceCommands {

    public static final AutonCommandGroup DRIVE_TO_LAST_CONE =
        new AutonCommandGroup(
            new DriveToPoint(-1.9, 0.3, 0.0, 0.8, 0.8, 0.2, 0.1, 5000),
            new DriveToPoint(-2.8, 0.3, 141.0, 0.7, 0.8, 0.3, 0.3, 5000),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.INTAKING_AUTO),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(ArmState.INTAKING_CONE, WristState.INTAKING_CONE),
            new DriveToPoint(-4.9, 1.75, 141.0, 0.0, 0.7, 0.2, 0.09, 2000),
            // moving to balance
            new DriveWait(),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.HOLDING_CONE));

    public static final AutonCommandGroup DRIVE_AND_BALANCE =
        new AutonCommandGroup(
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(ArmState.HOLDING, WristState.HOLDING),
            new DriveToPoint(-1.0, 2.55, 0.0, 0.8, 0.8, 0.5, 0.1, 2500),
            new DriveBalance(1, 0, 10000),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.SCORING_LOW),
            new DriveWait());
  }

  public static final class PreloadAndBalanceCommands {

    public static final AutonCommandGroup DRIVE_AND_BALANCE_MID =
        new AutonCommandGroup(
            new DriveToPoint(-4.2, 0, 0, 0.0, 0.4, 0.2, 0.1, 5000),
            new DriveToPoint(-2.6, 0, 0, 0.4, 0.4, 0.2, 0.1, 5000),
            new DriveBalance(1, 0, 10000),
            new DriveWait());

    public static final AutonCommandGroup DRIVE_AND_INTAKE_CUBE_MID =
        new AutonCommandGroup(
            new DriveToPoint(-2.6, 0, 0, 0.4, 0.4, 0.2, 0.1, 5000),
            new DriveToPoint(-4.6, 0.25, 175, 0.4, 0.4, 0.7, 0.1, 5000),
            new AutonSetPipeline(Pipeline.CUBE_TRACKING),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.INTAKING_AUTO),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(
                ArmState.INTAKING_CUBE,
                WristState.INTAKING_CUBE), // instead of arm wait so we will continue to hold
            new DriveToCube(0.3, 180.0, 0.7, true, 750),
            new DriveWait(),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeSetState(RollerState.HOLDING_CUBE),
            new AutonOverride(RobotComponent.ARM),
            new ArmWristSetState(
                ArmState.HOLDING,
                WristState.HOLDING), // instead of arm wait so we will continue to hold
            new DriveToPoint(-2.6, 0, 0, 0.4, 0.4, 0.7, 0.1, 5000),
            new DriveToPoint(-0.05, 0, 0, 0, 0.3, 0.7, 0.1, 5000),
            new DriveWait(),
            new AutonOverride(RobotComponent.INTAKE),
            new IntakeScoreLowCone(250),
            new IntakeWait(),
            new DriveToPoint(-2.3, 0, 0, 0.4, 0.5, 0.7, 0.1, 5000),
            new DriveBalance(1, 0, 10000),
            new DriveWait());
  }
}
