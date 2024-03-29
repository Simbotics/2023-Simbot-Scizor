package frc.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimPIDF extends SimPID {

  private double feedForward;

  public SimPIDF(double p, double i, double d, double f, double eps) {
    super(p, i, d, eps);
    this.feedForward = f;
  }

  @Override
  public double calcPID(double current) {
    double feedForwardOutput = (super.getDesiredVal() * this.feedForward);
    if (this.debug) {
      SmartDashboard.putNumber("FF out", feedForwardOutput);
    }
    return super.calcPID(current) + feedForwardOutput;
  }

  public void setConstants(double p, double i, double d, double f) {
    super.setConstants(p, i, d);
    this.feedForward = f;
  }

  @Override
  public boolean isDone() {
    double currError = Math.abs(this.previousError);

    // close enough to target
    // you don't need if(x) return true else return false
    // if the comparison is already true or false
    return currError <= this.finishedRange;
  }

  public void setFeedForward(double f) {
    this.feedForward = f;
  }
}
