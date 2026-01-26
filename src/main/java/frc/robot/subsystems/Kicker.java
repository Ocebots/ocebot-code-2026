package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.KickerConfig;

public class Kicker extends SubsystemBase {
    protected TalonFX kickerMotor;
    public Kicker() {
        kickerMotor = new TalonFX(CANMappings.KICKER_MOTOR_ID);
        TalonFXConfiguration kickerMotorConfig = new TalonFXConfiguration();

        kickerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kickerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kickerMotorConfig.CurrentLimits.SupplyCurrentLimit = 0;
        kickerMotorConfig.CurrentLimits.StatorCurrentLimit = 0;

        kickerMotorConfig.Feedback.SensorToMechanismRatio = KickerConfig.KICKER_GEAR_RATIO;
        kickerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        kickerMotor.getConfigurator().apply(kickerMotorConfig);
    }

    public void runKicker(double velocity){
        kickerMotor.setControl(new DutyCycleOut(velocity));
    }
    public void stopKicker(){
        kickerMotor.stopMotor();
    }

}
