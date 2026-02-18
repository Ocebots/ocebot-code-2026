package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.FlywheelConfig;

@Logged
public class Flywheel extends SubsystemBase {
  protected TalonFX flywheelRight;
  protected TalonFX flywheelLeft;

  public Flywheel() {
    flywheelRight = new TalonFX(CANMappings.FLYWHEEL_RIGHT_MOTOR_ID);
    flywheelLeft = new TalonFX(CANMappings.FLYWHEEL_LEFT_MOTOR_ID);

    TalonFXConfiguration flywheelRightConfig = new TalonFXConfiguration();
    TalonFXConfiguration flywheelLeftConfig = new TalonFXConfiguration();

    flywheelRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelRightConfig.CurrentLimits.StatorCurrentLimit =
        FlywheelConfig.FLYWHEEL_RIGHT_STATOR_CURRENT_LIMIT;

    flywheelLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelLeftConfig.CurrentLimits.StatorCurrentLimit =
        FlywheelConfig.FLYWHEEL_LEFT_STATOR_CURRENT_LIMIT;

    flywheelRightConfig.CurrentLimits.SupplyCurrentLimit =
        FlywheelConfig.FLYWHEEL_RIGHT_SUPPLY_CURRENT_LIMIT;

    flywheelLeftConfig.CurrentLimits.SupplyCurrentLimit =
        FlywheelConfig.FLYWHEEL_LEFT_SUPPLY_CURRENT_LIMIT;

    flywheelRightConfig.MotionMagic.MotionMagicCruiseVelocity =
        FlywheelConfig.FLYWHEEL_RIGHT_MAX_CRUISE_VELOCITY;
    flywheelRightConfig.MotionMagic.MotionMagicAcceleration =
        FlywheelConfig.FLYWHEEL_RIGHT_TARGET_ACCELERATION;

    flywheelLeftConfig.MotionMagic.MotionMagicCruiseVelocity =
        FlywheelConfig.FLYWHEEL_LEFT_MAX_CRUISE_VELOCITY;
    flywheelLeftConfig.MotionMagic.MotionMagicAcceleration =
        FlywheelConfig.FLYWHEEL_LEFT_TARGET_ACCELERATION;

    flywheelRightConfig.Slot0.kP = FlywheelConfig.FLYWHEEL_RIGHT_P;
    flywheelRightConfig.Slot0.kI = FlywheelConfig.FLYWHEEL_RIGHT_I;
    flywheelRightConfig.Slot0.kD = FlywheelConfig.FLYWHEEL_RIGHT_D;
    flywheelRightConfig.Slot0.kS = FlywheelConfig.FLYWHEEL_RIGHT_S;
    flywheelRightConfig.Slot0.kV = FlywheelConfig.FLYWHEEL_RIGHT_V;
    flywheelRightConfig.Slot0.kA = FlywheelConfig.FLYWHEEL_RIGHT_A;

    flywheelLeftConfig.Slot0.kP = FlywheelConfig.FLYWHEEL_LEFT_P;
    flywheelLeftConfig.Slot0.kI = FlywheelConfig.FLYWHEEL_LEFT_I;
    flywheelLeftConfig.Slot0.kD = FlywheelConfig.FLYWHEEL_LEFT_D;
    flywheelLeftConfig.Slot0.kS = FlywheelConfig.FLYWHEEL_LEFT_S;
    flywheelLeftConfig.Slot0.kV = FlywheelConfig.FLYWHEEL_LEFT_V;
    flywheelLeftConfig.Slot0.kA = FlywheelConfig.FLYWHEEL_LEFT_A;

    flywheelRightConfig.Feedback.SensorToMechanismRatio = FlywheelConfig.FLYWHEEL_RIGHT_GEAR_RATIO;
    flywheelRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelLeftConfig.Feedback.SensorToMechanismRatio = FlywheelConfig.FLYWHEEL_LEFT_GEAR_RATIO;
    flywheelLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelRight.getConfigurator().apply(flywheelRightConfig);
    flywheelLeft.getConfigurator().apply(flywheelLeftConfig);
  }

  public void shoot(double speed) {
    speed = Math.abs(speed);
    flywheelRight.setControl(new DutyCycleOut(speed));
    flywheelLeft.setControl(new DutyCycleOut(speed));
  }

  public void outtake(double speed) {
    speed = Math.abs(speed);
    flywheelRight.setControl(new DutyCycleOut(speed));
    flywheelLeft.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    flywheelRight.stopMotor();
    flywheelLeft.stopMotor();
  }

  public boolean atVelocity() {
    return (Math.abs(flywheelRight.getClosedLoopError().getValueAsDouble())
            <= FlywheelConfig.FLYWHEEL_TOLERANCE)
        && (Math.abs(flywheelLeft.getClosedLoopError().getValueAsDouble())
            <= FlywheelConfig.FLYWHEEL_TOLERANCE);
  }
}
