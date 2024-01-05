package frc.auton.util;

import frc.auton.AutonBase;

/**
 * Reverses auton elements to be used on other alliances
 *
 * @author Programmers
 * @since 2023
 */
public interface Reversible {
  public AutonBase reverse(ReversibleType type);
}
