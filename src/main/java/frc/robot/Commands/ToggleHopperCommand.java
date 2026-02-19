package frc.robot.Commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.HopperConfig;
import frc.robot.subsystems.Hopper;

@Logged
public class ToggleHopperCommand extends Command {
  private Hopper subsystem;
  private boolean hasExtended = false;

  public ToggleHopperCommand(Hopper subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (hasExtended) {
      // Retract
      subsystem.move(HopperConfig.HOPPER_RETRACT_ROTATION);
      System.out.println("Hopper: Retracting");
      hasExtended = false;
    } else {
      // Extend
      subsystem.move(HopperConfig.HOPPER_EXTEND_ROTATION);
      System.out.println("Hopper: Extending");
      hasExtended = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return subsystem.atPosition();
  }
}
