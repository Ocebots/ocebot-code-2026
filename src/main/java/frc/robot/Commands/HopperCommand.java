package frc.robot.Commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.HopperConfig;
import frc.robot.subsystems.Hopper;

@Logged
public class HopperCommand extends Command {
  public static enum Position {
    EXTEND_RETRACT,
    SHOOT_RETRACT_EXTEND
  }

  private Hopper subsystem;
  private HopperCommand.Position pose;

  public HopperCommand(Hopper subsystem, HopperCommand.Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {

      // (Incomplete) If hopper already extended, retract, and if hopper retracted, extend
      case EXTEND_RETRACT:
        subsystem.move(HopperConfig.HOPPER_EXTEND_ROTATION);
        System.out.println("Hopper: Extending/Retracting");
        break;

      // (Incomplete) Retracts then extends hopper back to original position to push balls into
      // kicker for shooting
      case SHOOT_RETRACT_EXTEND:
        subsystem.move(HopperConfig.HOPPER_RETRACT_ROTATION);
        System.out.println("Hopper: Retracting then Extending for Shots");
        break;

      default:
        break;
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
