package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.KickerConfig;

public class Kicker extends SubsystemBase {
  protected TalonFX kicker;

  public Kicker() {
    kicker = new TalonFX(CANMappings.KICKER_MOTOR_ID);
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    kickerConfig.CurrentLimits.SupplyCurrentLimit = KickerConfig.KICKER_SUPPLY_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.StatorCurrentLimit = KickerConfig.KICKER_STATOR_CURRENT_LIMIT;

    kickerConfig.Feedback.SensorToMechanismRatio = KickerConfig.KICKER_GEAR_RATIO;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kicker.getConfigurator().apply(kickerConfig);
  }

  public void intake(double speed) {
    speed = Math.abs(speed);
    kicker.setControl(new DutyCycleOut(speed));
  }

  public void outtake(double speed) {
    speed = Math.abs(speed);
    kicker.setControl(new DutyCycleOut(speed));
  }

  public void outtake() {
    kicker.setControl(new DutyCycleOut(KickerConfig.KICKER_OUTTAKE_SPEED));
  }

  public void stop() {
    kicker.stopMotor();
  }
}
