package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.IntakeConfig;
import frc.robot.subsystems.Intake;

@Logged
public class IntakeCommand extends Command {
  public static enum Position {
    INTAKE,
    OUTTAKE,
    SLOW_INTAKE
  }

  private Intake subsystem;
  private IntakeCommand.Position pose;

  public IntakeCommand(Intake subsystem, IntakeCommand.Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      // Runs intake
      case INTAKE:
        subsystem.intake(IntakeConfig.INTAKE_INTAKE_SPEED);
//        System.out.println("Intake: Intaking");
        break;

      // Runs intake in outtaking direction
      case OUTTAKE:
        subsystem.outtake(IntakeConfig.INTAKE_OUTTAKE_SPEED);
        break;

      // Slow intake for shooting
      case SLOW_INTAKE:
        subsystem.intake(IntakeConfig.INTAKE_SLOW_SPEED);
//        System.out.println("Intake: run for shooting");
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
