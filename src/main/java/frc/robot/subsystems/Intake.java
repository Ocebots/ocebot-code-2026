package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.IntakeConfig;

@Logged
public class Intake extends SubsystemBase {
  protected TalonFX intake;
//  protected TalonFX swerveDrive;
//  protected TalonFX swerveSteer;
  public static String intakeState = "Stopped";

  public Intake() {
    intake = new TalonFX(CANMappings.INTAKE_MOTOR_ID);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

//    swerveDrive = new TalonFX(31);
//    swerveSteer = new TalonFX(30);
//    TalonFXConfiguration swerveConfig = new TalonFXConfiguration();
//    swerveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//    swerveDrive.getConfigurator().apply(swerveConfig);
//    swerveSteer.getConfigurator().apply(swerveConfig);

    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConfig.INTAKE_SUPPLY_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConfig.INTAKE_STATOR_CURRENT_LIMIT;

    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConfig.INTAKE_GEAR_RATIO;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intake.getConfigurator().apply(intakeConfig);
  }

  public void intake(double speed) {
    speed = Math.abs(speed);
    intake.setControl(new VoltageOut(speed));
    if (speed == IntakeConfig.INTAKE_INTAKE_SPEED) {
      intakeState = "Intaking: Default";
    } else if (speed == IntakeConfig.INTAKE_SLOW_SPEED) {
      intakeState = "Intaking: Slow";
    } else {
      intakeState = "Intaking: Unknown";
    }
  }

  public void outtake(double speed) {
    speed = Math.abs(speed);
    intake.setControl(new VoltageOut(-speed));
    intakeState = "Outtaking";
  }

  public void stop() {
    intake.stopMotor();
    intakeState = "Stopped";
  }
}
