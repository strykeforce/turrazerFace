package frc.robot.constants;

import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TurretConstants {

public static final Angle leftTurretLimit = null;
public static final Angle rightTurretLimit = null;

public static TalonFXConfiguration turretFXConfig() {
    TalonFXConfiguration fxConfig = new TalonFXConfiguration();

     MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(70)
            .withMotionMagicAcceleration(300) // was 300
            .withMotionMagicJerk(1500);
    fxConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    fxConfig.MotorOutput = motorOut;
    return fxConfig;
}
    
}