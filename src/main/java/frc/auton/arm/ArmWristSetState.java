package frc.auton.arm;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Arm;

public class ArmWristSetState extends AutonCommand {

  // Define arm and arm state that isn't initialized
  private Arm arm;
  private Arm.ArmState armState;
  private Arm.WristState wristState;

  private IO io;

  public ArmWristSetState(Arm.ArmState armState, Arm.WristState wristState) {
    super(RobotComponent.ARM);

    // Initialize/define the undefined variables
    this.arm = Arm.getInstance();
    this.armState = armState;
    this.wristState = wristState;
    this.io = IO.getInstance();
  }

  @Override
  public void firstCycle() {
    arm.setArmWristState(this.armState, this.wristState);
  }

  @Override
  public boolean calculate() {
    arm.calculate();
    return false;
  }

  @Override
  public void override() {
    // When this is triggered, set the motor speeds to 0 for the arm and wrist
    this.io.setDistalArm(0);
    this.io.setProximalArm(0);
    this.io.setWristMotor(0);
  }
}
