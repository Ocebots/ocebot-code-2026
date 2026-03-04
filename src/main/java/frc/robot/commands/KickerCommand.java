package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.KickerConfig;
import frc.robot.subsystems.Kicker;

@Logged
public class KickerCommand extends Command {
  public static enum Position {
    INTAKE,
    OUTTAKE
  }

  private Kicker subsystem;
  private Position pose;

  public KickerCommand(Kicker subsystem, Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      // Intakes balls to shooter
      case INTAKE:
        subsystem.intake(KickerConfig.KICKER_INTAKE_SPEED);
        System.out.println("Kicker: Intaking");
        break;

      // Pushes balls out of shooter area to hopper area
      case OUTTAKE:
        subsystem.outtake(KickerConfig.KICKER_OUTTAKE_SPEED);
        System.out.println("Kicker: Outtaking");
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
    return false;
  }
}
