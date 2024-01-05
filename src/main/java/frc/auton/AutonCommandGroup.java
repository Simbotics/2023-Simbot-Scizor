package frc.auton;

import frc.auton.mode.AutonBuilder;
import frc.auton.util.Reversible;
import frc.auton.util.ReversibleType;
import java.util.ArrayList;
import java.util.List;

/**
 * A group of autonomous components, including other groups
 *
 * @author Programmers
 * @since 2023
 */
public class AutonCommandGroup implements AutonBase, Reversible {
  private List<AutonBase> commands = new ArrayList<>();

  public AutonCommandGroup(AutonBase... commands) {
    this.commands = List.of(commands);
  }

  public AutonCommandGroup(List<AutonBase> commands) {
    this.commands = commands;
  }

  @Override
  public void addToAutonBuilder(AutonBuilder ab) {
    for (AutonBase command : commands) {
      command.addToAutonBuilder(ab);
    }
  }

  /**
   * Reverses auto group for different sides
   *
   * @see Reversible
   * @return an auton group with every reversable command flipped
   * @since 2023
   */
  @Override
  public AutonCommandGroup reverse(ReversibleType type) {
    List<AutonBase> updatedCommands = new ArrayList<>();

    for (AutonBase command : this.commands) {
      if (command instanceof Reversible) {
        updatedCommands.add(((Reversible) command).reverse(type));
      } else {
        updatedCommands.add(command);
      }
    }
    return new AutonCommandGroup(updatedCommands);
  }
}
