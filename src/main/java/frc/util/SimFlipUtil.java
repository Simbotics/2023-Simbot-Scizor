// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
public class SimFlipUtil {
  /** Flips a translation to the correct side of the field based on the current alliance color. */

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {

    return new Pose2d(pose.getX(), -pose.getY(), pose.getRotation());
  }
}
