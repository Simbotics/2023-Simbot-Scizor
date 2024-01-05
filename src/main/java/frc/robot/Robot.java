package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.auton.AutonControl;
import frc.io.Dashboard;
import frc.io.IO;
import frc.io.Logger;
import frc.subsystems.LEDStrip;
import frc.teleop.TeleopControl;
import frc.util.Debugger;

public class Robot extends TimedRobot {

  private LEDStrip leds;
  private IO io;
  private TeleopControl teleopControl;
  private Logger logger;
  private Dashboard dashboard;
  private boolean pushToDashboard = true;
  public static boolean teleopInitialized = false;
  private Timer garbageCollector = new Timer();

  public Robot() {
    super(0.01);
  }

  @Override
  public void robotInit() {
    Debugger.defaultOn();
    this.io = IO.getInstance();
    this.dashboard = Dashboard.getInstance();
    garbageCollector.start();
    if (this.pushToDashboard) {
      RobotConstants.pushValues();
    }

    this.teleopControl = TeleopControl.getInstance();

    this.logger = Logger.getInstance();
    LiveWindow.setEnabled(false);
    LiveWindow.disableAllTelemetry();
    this.teleopControl.initialize();

    this.leds = LEDStrip.getInstance();
  }

  @Override
  public void robotPeriodic() {
    this.leds.update();
    this.io.update();
    this.dashboard.updateAll();
  }

  @Override
  public void disabledInit() {
    this.io.stopAll();
    this.leds.setDefault();
    this.teleopControl.disable();
    this.logger.close();
  }

  @Override
  public void disabledPeriodic() {
    AutonControl.getInstance().updateModes();

    if (garbageCollector.hasElapsed(15)) {
      System.gc();
      garbageCollector.restart();
    }
  }

  @Override
  public void autonomousInit() {

    AutonControl.getInstance().initialize();
    AutonControl.getInstance().setRunning(true);
    AutonControl.getInstance().setOverrideAuto(false);
    this.io.setLimelightPipeline(0);
    this.io.reset();
    this.io.setIsAuto(true);
    this.io.resetAutonTimer();
    // this.logger.openFile();

  }

  @Override
  public void autonomousPeriodic() {
    AutonControl.getInstance().runCycle();
    // this.logger.logAll();
  }

  public void testInit() {}

  public void testPeriodic() {}

  @Override
  public void teleopInit() {

    this.teleopControl.initialize();
    this.io.setIsAuto(false);
    this.io.setOdometryUsingLimelight(false);
    this.io.setLimelightPipeline(1);
    Robot.teleopInitialized = true;
  }

  @Override
  public void teleopPeriodic() {
    this.teleopControl.runCycle();
    if (garbageCollector.hasElapsed(15)) {
      System.gc();
      garbageCollector.restart();
    }
  }
}
