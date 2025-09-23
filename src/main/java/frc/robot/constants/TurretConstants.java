package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  // TODO Add real constants
  public static final Angle kLeftTurretLimit = null;
  public static final Angle kRightTurretLimit = null;
  public static final double kTurretCloseEnough = 0;

  public static final int kTurretFxId = 0;
  public static final int kCanCoder1Id = 21;
  public static final int kCanCoder2Id = 22;
  public static final double kCanCoder1Gr = 20 / 150;
  public static final double kCanCoder2Gr = 19 / 150;
  public static final double kBigGr = 1 / 100;

  public static final double kCanCoder1Zero = 0;
  public static final double kCanCoder2Zero = 0;

  public static final double kFxForwardMax = 0;
  public static final double kFxReverseMax = 0;
  // List of gear rations gr, 2gr, 3gr and so on
  public static final double[] kCanCoder1PosRot = {};
  public static final double[] kCanCoder2PosRot = {};

  public static final double kZeroDifferenceTol = 100;

  public static TalonFXConfiguration turretFXConfig() {
    TalonFXConfiguration fxConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(70)
            .withSupplyCurrentLowerLimit(10)
            .withSupplyCurrentLowerTime(2);
    fxConfig.CurrentLimits = current;

    HardwareLimitSwitchConfigs hwLimit =
        new HardwareLimitSwitchConfigs()
            .withForwardLimitAutosetPositionEnable(false)
            .withForwardLimitEnable(false)
            .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
            .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
            .withReverseLimitAutosetPositionEnable(false)
            .withReverseLimitEnable(false)
            .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin);
    fxConfig.HardwareLimitSwitch = hwLimit;

    SoftwareLimitSwitchConfigs swLimit =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kFxForwardMax)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(kFxReverseMax);
    fxConfig.SoftwareLimitSwitch = swLimit;

    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(2)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0.36)
            .withKS(0)
            .withKV(0.13)
            .withKA(0);
    fxConfig.Slot0 = slot0;

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
