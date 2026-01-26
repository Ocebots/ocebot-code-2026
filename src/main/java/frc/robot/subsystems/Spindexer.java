package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.SpindexerConfig;

public class Spindexer extends SubsystemBase {
  protected TalonFX spindexerMotor;

  public Spindexer() {
    spindexerMotor = new TalonFX(CANMappings.SPINDEXER_MOTOR_ID);
    TalonFXConfiguration spindexerMotorConfig = new TalonFXConfiguration();

    spindexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    spindexerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    spindexerMotorConfig.CurrentLimits.SupplyCurrentLimit =
        SpindexerConfig.SPINDEXER_SUPPLY_CURRENT_LIMIT;
    spindexerMotorConfig.CurrentLimits.StatorCurrentLimit =
        SpindexerConfig.SPINDEXER_STATOR_CURRENT_LIMIT;

    spindexerMotorConfig.Feedback.SensorToMechanismRatio = SpindexerConfig.SPINDEXER_GEAR_RATIO;
    spindexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    spindexerMotor.getConfigurator().apply(spindexerMotorConfig);
  }

  public void index(double velocity) {
    spindexerMotor.setControl(new DutyCycleOut(velocity));
  }

  public void stop() {
    spindexerMotor.stopMotor();
  }
}
