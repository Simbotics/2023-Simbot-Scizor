package frc.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.mode.AutonBuilder;
import frc.auton.mode.AutonMode;
import frc.auton.mode.Blue2PieceBalance;
import frc.auton.mode.BlueBumpSide3PieceStemley;
import frc.auton.mode.BlueHPSide3PieceStemley;
import frc.auton.mode.BlueMiddle1ConeBalance;
import frc.auton.mode.DefaultMode;
import frc.auton.mode.DriveBalanceTest;
import frc.auton.mode.Red2PieceBalance;
import frc.auton.mode.RedBumpSide3PieceStemley;
import frc.auton.mode.RedHPSide3PieceStemley;
import frc.auton.mode.RedMiddle1ConeBalance;
import frc.io.DriverInput;
import frc.io.IO;
import frc.robot.Robot;
import frc.teleop.TeleopControl;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;

/**
 * @author Programmers
 */
public class AutonControl {

  private static AutonControl instance;
  private TeleopControl teleopControl;
  private DriverInput driverIn;
  private boolean OverrideAuto = false;
  private boolean hasTapped = false;
  private boolean released = false;
  private long doubleTapLength = 500;
  private long timeOfFirstTap;
  private long timeOfTap;

  public static final int NUM_ARRAY_MODE_STEPS = 1;

  private int autonDelay;
  private long autonStartTime;

  private boolean running = false;

  private int curAutonStepToSet = 0;
  private int[] autonSubmodeSelections = new int[NUM_ARRAY_MODE_STEPS];
  private ArrayList<ArrayList<AutonMode>> autonSteps = new ArrayList<>();

  private int currIndex;
  private AutonCommand[] commands;

  public static AutonControl getInstance() {
    if (instance == null) {
      instance = new AutonControl();
    }
    return instance;
  }

  private AutonControl() {
    this.teleopControl = TeleopControl.getInstance();
    this.driverIn = DriverInput.getInstance();
    this.autonDelay = 0;
    this.currIndex = 0;

    for (int i = 0; i < NUM_ARRAY_MODE_STEPS; i++) {
      this.autonSteps.add(new ArrayList<AutonMode>());
      this.autonSubmodeSelections[i] = 0; // default to default auto modes
    }

    ArrayList<AutonMode> step1 = this.autonSteps.get(0);

    step1.add(new DefaultMode());

    step1.add(new RedMiddle1ConeBalance());
    step1.add(new Red2PieceBalance());
    step1.add(new RedHPSide3PieceStemley());
    step1.add(new RedBumpSide3PieceStemley());

    step1.add(new BlueMiddle1ConeBalance());
    step1.add(new Blue2PieceBalance());
    step1.add(new BlueHPSide3PieceStemley());
    step1.add(new BlueBumpSide3PieceStemley());

    step1.add(new DriveBalanceTest());
  }

  public void initialize() {
    System.out.println("START AUTO");

    this.currIndex = 0;
    this.running = true;

    // initialize auton in runCycle
    AutonBuilder ab = new AutonBuilder();

    // add auton commands from all the different steps
    for (int i = 0; i < this.autonSteps.size(); i++) {
      this.autonSteps.get(i).get(this.autonSubmodeSelections[i]).addToMode(ab);
    }

    // get the full auton mode
    this.commands = ab.getAutonList();

    this.autonStartTime = System.currentTimeMillis();

    // clear out each components "run seat"
    AutonCommand.reset();
  }

  public void runCycle() {
    // haven't initialized list yet
    long timeElapsed = System.currentTimeMillis() - this.autonStartTime;

    if (!this.hasTapped) {
      if (this.driverIn.getDriverAutoOverrideButtons()
          || this.driverIn.getOperatorAutoOverrideButtons()) {
        this.hasTapped = true;
        this.released = false;
        this.timeOfFirstTap = System.currentTimeMillis();
      }
    } else {
      if (!this.driverIn.getDriverAutoOverrideButtons()
          && !this.driverIn.getOperatorAutoOverrideButtons()) {
        this.released = true;
      }
    }

    if (this.released) {
      if (this.driverIn.getDriverAutoOverrideButtons()
          || this.driverIn.getOperatorAutoOverrideButtons()) {
        this.timeOfTap = System.currentTimeMillis() - this.timeOfFirstTap;
        if (this.timeOfTap < this.doubleTapLength) {
          this.OverrideAuto = true;
          this.teleopControl.initialize();
          Robot.teleopInitialized = true;
        } else {
          this.hasTapped = true;
          this.timeOfFirstTap = System.currentTimeMillis();
          this.released = false;
        }
      }
    }

    if (this.OverrideAuto) {
      this.running = false;
      this.teleopControl.runCycle();
    } else {
      if (timeElapsed > this.getAutonDelayLength() && this.running) {
        // System.out.println("Current index " + this.currIndex);

        // start waiting commands
        while (this.currIndex < this.commands.length
            && this.commands[this.currIndex].checkAndRun()) {
          this.currIndex++;
        }
        // calculate call for all running commands
        AutonCommand.execute();
      } else {
        IO.getInstance().stopAll();
      }
    }
  }

  public void stop() {
    this.running = false;
  }

  public long getAutonDelayLength() {
    return this.autonDelay * 500;
  }

  public void updateModes() {
    DriverInput driverIn = DriverInput.getInstance();

    if (driverIn.getAutonStepIncrease()) {
      this.curAutonStepToSet++;
      this.curAutonStepToSet = Math.min(this.curAutonStepToSet, this.autonSteps.size() - 1);
    }

    if (driverIn.getAutonStepDecrease()) {
      this.curAutonStepToSet--;
      this.curAutonStepToSet = Math.max(this.curAutonStepToSet, 0);
    }

    boolean updatingAutoMode = false;

    try {

      int val = 0;
      if (driverIn.getAutonModeIncrease()) {
        val = 1;
      } else if (driverIn.getAutonModeIncreaseBy10()) {
        val = 10;
      } else if (driverIn.getAutonModeDecrease()) {
        val = -1;
      } else if (driverIn.getAutonModeDecreaseBy10()) {
        val = -10;
      }

      if (val != 0) {
        updatingAutoMode = true;

        // figure out which auton mode is being selected
        int autonMode = this.autonSubmodeSelections[this.curAutonStepToSet] + val;

        // make sure we didn't go off the end of the list
        autonMode = Math.min(autonMode, this.autonSteps.get(this.curAutonStepToSet).size() - 1);
        if (autonMode < 0) {
          autonMode = 0;
        }

        this.autonSubmodeSelections[this.curAutonStepToSet] = autonMode;

        /*
         * if(val < 0) { this.autonMode = 0; } else { this.autonMode = 1; }
         */
      } else if (driverIn.getAutonSetDelayButton()) {
        this.autonDelay = (int) ((driverIn.getAutonDelayStick() + 1) * 5.0);
        if (this.autonDelay < 0) {
          this.autonDelay = 0;
        }
      }

    } catch (Exception e) {
      // this.autonMode = 0;

      StringWriter sw = new StringWriter();
      e.printStackTrace(new PrintWriter(sw));
    }

    // display steps of auto
    for (int i = 0; i < autonSteps.size(); i++) {
      // name of the current auton mode
      String name = this.autonSteps.get(i).get(this.autonSubmodeSelections[i]).getClass().getName();

      // make sure there is a '.'
      if (name.lastIndexOf('.') >= 0) {
        // get just the last bit of the name
        name = name.substring(name.lastIndexOf('.'));
      }

      String outputString = "" + autonSubmodeSelections[i] + name + "";

      SmartDashboard.putString("12_Auton Step " + (i + 1) + ": ", outputString);

      if (updatingAutoMode) {
        // System.out.print(this.autonSubmodeSelections[i] + "-");
        System.out.println("Step " + (i + 1) + ": " + outputString);
      }

      // System.out.println();

      // SmartDashboard.putString("Auton Error: ", this.autoSelectError);
    }

    if (updatingAutoMode) {
      System.out.println("----------------------------------");
    }

    // step we are currently modifying
    SmartDashboard.putNumber("12_SETTING AUTON STEP: ", this.curAutonStepToSet + 1);

    // delay

    SmartDashboard.putNumber("12_Auton Delay: ", this.autonDelay);
  }

  public boolean isRunning() {
    return this.running;
  }

  public void setRunning(boolean isRunning) {
    this.running = isRunning;
  }

  public void setOverrideAuto(boolean overrideAuto) {
    this.OverrideAuto = overrideAuto;
  }
}
