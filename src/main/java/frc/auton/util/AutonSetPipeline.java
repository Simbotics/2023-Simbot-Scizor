package frc.auton.util;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;

/**
 * @author Billy bob susan
 */
public class AutonSetPipeline extends AutonCommand {

  public enum Pipeline {
    APRIL_TAG,
    HUMAN_LOADING,
    CUBE_TRACKING
  }

  private Pipeline pipeline;

  public AutonSetPipeline(Pipeline pipeline) {
    super(RobotComponent.UTIL);
    this.pipeline = pipeline;
  }

  @Override
  public void firstCycle() {
    switch (this.pipeline) {
      case APRIL_TAG:
        IO.getInstance().setLimelightPipeline(0);
        break;
      case CUBE_TRACKING:
        IO.getInstance().setLimelightPipeline(2);
        break;
      case HUMAN_LOADING:
        IO.getInstance().setLimelightPipeline(1);
        break;
      default:
        IO.getInstance().setLimelightPipeline(0);
        break;
    }
  }

  @Override
  public boolean calculate() {
    return true;
  }

  @Override
  public void override() {
    // nothing to do

  }
}
