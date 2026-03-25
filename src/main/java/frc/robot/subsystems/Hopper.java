package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.HopperConfig;

@Logged
public class Hopper extends SubsystemBase {
  protected TalonFX hopper;
  public static String hopperState = "None";

  public Hopper() {

    hopper = new TalonFX(CANMappings.HOPPER_MOTOR_ID);

    TalonFXConfiguration hopperConfig = new TalonFXConfiguration();

    hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hopperConfig.CurrentLimits.StatorCurrentLimit = HopperConfig.HOPPER_STATOR_CURRENT_LIMIT;

    hopperConfig.CurrentLimits.SupplyCurrentLimit = HopperConfig.HOPPER_SUPPLY_CURRENT_LIMIT;

    hopperConfig.MotionMagic.MotionMagicCruiseVelocity = HopperConfig.HOPPER_MAX_CRUISE_VELOCITY;
    hopperConfig.MotionMagic.MotionMagicAcceleration = HopperConfig.HOPPER_TARGET_ACCELERATION;

    hopperConfig.Slot0.kP = HopperConfig.HOPPER_P;
    hopperConfig.Slot0.kI = HopperConfig.HOPPER_I;
    hopperConfig.Slot0.kD = HopperConfig.HOPPER_D;
    hopperConfig.Slot0.kS = HopperConfig.HOPPER_S;
    hopperConfig.Slot0.kV = HopperConfig.HOPPER_V;
    hopperConfig.Slot0.kA = HopperConfig.HOPPER_A;

    hopperConfig.Slot1.kP = HopperConfig.SLOW_HOPPER_P;
    hopperConfig.Slot1.kI = HopperConfig.SLOW_HOPPER_I;
    hopperConfig.Slot1.kD = HopperConfig.SLOW_HOPPER_D;

    hopperConfig.Feedback.SensorToMechanismRatio = HopperConfig.HOPPER_GEAR_RATIO;

    hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hopper.getConfigurator().apply(hopperConfig);
  }

  public void move(double rotation) {
    hopper.setControl(new MotionMagicVoltage(rotation).withSlot(0));
    if (rotation == HopperConfig.HOPPER_EXTEND_ROTATION) {
      hopperState = "Move: Extending";
    } else if (rotation == HopperConfig.HOPPER_RETRACT_ROTATION) {
      hopperState = "Move: Retracting";
    } else {
      hopperState = "Move: Unknown";
    }
  }

  public void slowMove(double rotation) {
    hopper.setControl(new MotionMagicVoltage(rotation).withSlot(1));
    if (rotation == HopperConfig.HOPPER_EXTEND_ROTATION) {
      hopperState = "Slow Move: Extending";
    } else if (rotation == HopperConfig.HOPPER_RETRACT_ROTATION) {
      hopperState = "Slow Move: Retracting";
    } else {
      hopperState = "Slow Move: Unknown";
    }
  }

  public void extendDirectional(double speed) {
    speed = Math.abs(speed);
    hopper.setControl(new VoltageOut(-speed));
    hopperState = "Extend Directional";
  }

  public void retractDirectional(double speed) {
    speed = Math.abs(speed);
    hopper.setControl(new VoltageOut(-speed));
    hopperState = "Retract Directional";
  }

  public void stop() {
    hopper.stopMotor();
    hopperState = "Stopped";
  }

  public void zero() {
    hopper.setPosition(0.0);
  }

  public boolean atPosition() {
    return (Math.abs(hopper.getClosedLoopError().getValueAsDouble())
        <= HopperConfig.HOPPER_TOLERANCE);
  }

  public boolean isExtendedByPosition() {
    return (Math.abs(hopper.getPosition().getValueAsDouble() - HopperConfig.HOPPER_EXTEND_ROTATION)
        <= HopperConfig.HOPPER_TOLERANCE);
  }

  public boolean isRetractedByPosition() {
    return (hopper.getPosition().getValueAsDouble()
        < HopperConfig.HOPPER_RETRACT_FOR_NEUTRAL_MODE_ROTATION);
  }

  private boolean extended = false;

  public boolean isExtended() {
    return this.extended;
  }

  public static double getRotation(boolean isExtended) {
    if (isExtended) {
      return HopperConfig.HOPPER_RETRACT_ROTATION;
    } else {
      return HopperConfig.HOPPER_EXTEND_ROTATION;
    }
  }

  public void toggleExtend() {
    if (extended) {
      move(HopperConfig.HOPPER_RETRACT_ROTATION);
      extended = false;
    } else {
      move(HopperConfig.HOPPER_EXTEND_ROTATION);
      extended = true;
    }
  }
}
