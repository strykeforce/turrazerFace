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

/*
 * kp 8.00
 * kD 0.2
 * kV 0.095
 * Motion Magic
 * A 300
 * CV 120
 * kA .1
 * kV .12
 * Current Limits
 * scl 50
 * supply 20
 * 120 rot = 360
 * 130 max speed
 * Cw positive
 * 2 cancoders CW positive
 */
public class TurretConstants {
  // TODO Add real constants
  public static final Angle kLeftTurretLimit = null;
  public static final Angle kRightTurretLimit = null;
  public static final double kTurretCloseEnough = 0.001;

  public static final int kTurretFxId = 0;
  public static final int kCanCoder1Id = 21;
  public static final int kCanCoder2Id = 22;
  public static final double kCanCoder1Gr = 20 / 150;
  public static final double kCanCoder2Gr = 19 / 150;
  public static final double kBigGr = 1 / 150;
  public static final double kMotorGr = (38 / 150) * (1 / 30);

  public static final double kCanCoder1Zero = 0;
  public static final double kCanCoder2Zero = 0;

  public static final double kWrap1 = -90 / kMotorGr;
  public static final double kWrap2 = 360 / kMotorGr;
  public static final double kWrapTooClose = 1;

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
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLowerLimit(50)
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
            .withKP(8)
            .withKI(0)
            .withKD(0.2)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0.0)
            .withKS(0)
            .withKV(0.095)
            .withKA(0);
    fxConfig.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(120)
            .withMotionMagicAcceleration(300) // was 300
            .withMotionMagicJerk(0);
    fxConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
    fxConfig.MotorOutput = motorOut;
    return fxConfig;
  }
}
