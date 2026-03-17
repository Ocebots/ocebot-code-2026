package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.config.HopperConfig;
import frc.robot.subsystems.Hopper;

@Logged
public class HopperCommand extends Command {
  public static enum Position {
    EXTEND_RETRACT,
    SHOOT_RETRACT,
    SHOOT_EXTEND
  }

  private Hopper subsystem;
  private HopperCommand.Position pose;
  private boolean isExtended;

  public HopperCommand(Hopper subsystem, HopperCommand.Position pose, boolean isExtended) {
    this.pose = pose;
    this.subsystem = subsystem;
    this.isExtended = isExtended;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {

      // (Needs check) If hopper already extended, retract, and if hopper retracted, extend
      case EXTEND_RETRACT:
        if (isExtended) {
          System.out.println("Hopper: Retracting");
          subsystem.move(HopperConfig.HOPPER_RETRACT_ROTATION);
          RobotContainer.isExtended = false;
          System.out.println(RobotContainer.isExtended);
        } else {
          System.out.println("Hopper: Extending");
          subsystem.move(HopperConfig.HOPPER_EXTEND_ROTATION);
          RobotContainer.isExtended = true;
          System.out.println(RobotContainer.isExtended);
        }
        break;

      // (Needs check) Retracts then extends hopper back to original position to push balls into
      // kicker for shooting
      case SHOOT_RETRACT:
        //        System.out.println("Hopper: Shoot Retracting");
        subsystem.slowMove(HopperConfig.HOPPER_RETRACT_ROTATION);
        break;

      case SHOOT_EXTEND:
        //        System.out.println("Hopper: Shoot Extending");
        subsystem.move(HopperConfig.HOPPER_EXTEND_ROTATION);
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
