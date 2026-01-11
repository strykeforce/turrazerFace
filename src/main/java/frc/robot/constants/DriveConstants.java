package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveConstants {
  public static final double kAlgaeRemovalSpeed = 0;
  public static final double kBargeScoreStickMultiplier = 0.5;

  public static final double kDeadbandAllStick = 0.075;
  public static final double kExpoScaleYawFactor = 0.75;
  public static final double kRateLimitFwdStr = 3.5;
  public static final double kRateLimitYaw = 8.0;

  public static final double kDriveMotorOutputGear = 22;
  public static final double kDriveInputGear = 52;
  public static final double kBevelInputGear = 15;
  public static final double kBevelOutputGear = 45;

  public static final double kDriveGearRatio =
      (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

  public static final double kWheelDiameterInches = 3.375;
  public static final double kMaxSpeedMetersPerSecond = 3.782;
  public static final double kSpeedStillThreshold = 0.2; // meters per second
  public static final double kGyroRateStillThreshold = 10.0; // was 10
  public static final double kGyroDifferentThreshold = 0.3; // 5 degrees
  public static final int kGyroDifferentCount = 3;

  public static final double kRobotLength = 0.61595;
  public static final double kRobotWidth = 0.61595;
  public static final double kFieldMaxX = 17.526;
  public static final double kFieldMaxY = 8.0518;
  public static final double kCenterLineX = 8.763;

  public static final double kPOmega = 6.0;
  public static final double kIOmega = 0.0;
  public static final double kDOmega = 0.0;
  public static final double kMaxAutoOmega = 10000;
  public static final double kMaxVelOmega =
      (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0)) / 2.0;
  public static final double kMaxAccelOmega = 100000; // was 5

  public static final double kPHolonomic = 4.0; // was 3
  public static final double kIHolonomic = 0.0000;
  public static final double kDHolonomic = 0.00; // kPHolonomic/100

  public static Translation2d[] getWheelLocationMeters() {
    final double x = kRobotLength / 2.0; // front-back, was ROBOT_LENGTH
    final double y = kRobotWidth / 2.0; // left-right, was ROBOT_WIDTH
    Translation2d[] locs = new Translation2d[4];
    locs[0] = new Translation2d(x, y); // left front
    locs[1] = new Translation2d(x, -y); // right front
    locs[2] = new Translation2d(-x, y); // left rear
    locs[3] = new Translation2d(-x, -y); // right rear
    return locs;
  }

  // temp stuff
  public static final int kTempAvgCount = 25;
  public static final double kTripTemp = 1300;
  public static final double kRecoverTemp = 1290;
  public static final double kNotifyTemp = 1295;

  public static final Pose2d kResetOdomPose =
      new Pose2d(new Translation2d(0.5, 3.62), Rotation2d.fromDegrees(67));

  public static TalonFXSConfiguration getAzimuthFXSConfig() {
    // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
    TalonFXSConfiguration azimuthConfig = new TalonFXSConfiguration();

    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;
    azimuthConfig.HardwareLimitSwitch = hardwareLimitSwitchConfigs;

    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 20;
    currentConfig.SupplyCurrentLowerTime = 20;
    currentConfig.SupplyCurrentLowerLimit = 1;

    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = false;

    azimuthConfig.CurrentLimits = currentConfig;

    Slot0Configs slot0Config = new Slot0Configs();
    slot0Config.kP = 300;
    slot0Config.kI = 0.0;
    slot0Config.kD = 3;
    slot0Config.kV = 4.3;

    azimuthConfig.Slot0 = slot0Config;

    ExternalFeedbackConfigs externalFeedbackConfigs = new ExternalFeedbackConfigs();
    externalFeedbackConfigs.VelocityFilterTimeConstant = 0.02; // ?
    externalFeedbackConfigs.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.Quadrature;
    externalFeedbackConfigs.SensorPhase = SensorPhaseValue.Opposed;
    azimuthConfig.ExternalFeedback = externalFeedbackConfigs;

    // VoltageConfigs voltageConfig = new VoltageConfigs();
    // voltageConfig.SupplyVoltageTimeConstant = 3.2; // FIXME, seems very long
    // azimuthConfig.Voltage = voltageConfig;

    MotionMagicConfigs motionConfig = new MotionMagicConfigs();
    motionConfig.MotionMagicCruiseVelocity = 2;
    motionConfig.MotionMagicAcceleration = 20;
    azimuthConfig.MotionMagic = motionConfig;

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    // motorConfigs.DutyCycleNeutralDeadband = 0.04;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    azimuthConfig.MotorOutput = motorConfigs;

    CommutationConfigs commutationConfigs = new CommutationConfigs();
    commutationConfigs.MotorArrangement = MotorArrangementValue.Minion_JST;

    azimuthConfig.Commutation = commutationConfigs;

    return azimuthConfig;
  }

  public static TalonSRXConfiguration getAzimuthTalonConfig() {
    // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();

    azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

    azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
    azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;

    azimuthConfig.slot0.kP = 15.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 150.0;
    azimuthConfig.slot0.kF = 1.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.slot0.maxIntegralAccumulator = 0;

    azimuthConfig.motionCruiseVelocity = 800;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.velocityMeasurementWindow = 64;
    azimuthConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    azimuthConfig.voltageCompSaturation = 12;
    azimuthConfig.voltageMeasurementFilter = 32;
    azimuthConfig.neutralDeadband = 0.04;
    return azimuthConfig;
  }

  public static TalonFXConfiguration getDriveTalonConfig() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 80;
    currentConfig.SupplyCurrentLowerLimit = 80;
    currentConfig.SupplyCurrentLowerTime = 1.0;
    currentConfig.StatorCurrentLimit = 160;

    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = true;

    driveConfig.CurrentLimits = currentConfig;

    Slot0Configs slot0Config = new Slot0Configs();
    slot0Config.kP = 0.2;
    slot0Config.kI = 0.0;
    slot0Config.kD = 0.0;
    slot0Config.kV = 0.117;
    slot0Config.kS = 0.0;
    slot0Config.kA = 0.0;
    slot0Config.kG = 0.0;
    driveConfig.Slot0 = slot0Config;

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.DutyCycleNeutralDeadband = 0.01;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput = motorConfigs;

    return driveConfig;
  }

  public static final int kPigeonCanID = 4;

  public static Pigeon2Configuration getPigeon2Configuration() {
    Pigeon2Configuration config = new Pigeon2Configuration();

    config.MountPose.MountPoseYaw = 0.0;
    config.MountPose.MountPoseRoll = 0.0;
    config.MountPose.MountPosePitch = 0.0;

    config.GyroTrim.GyroScalarX = -1.2;
    config.GyroTrim.GyroScalarY = 4.8;
    config.GyroTrim.GyroScalarZ = -2.9;

    return config;
  }

  public static Pigeon2Configuration getPigeon2NoMotionDisabledConfiguration() {
    Pigeon2FeaturesConfigs featureConfigs =
        new Pigeon2FeaturesConfigs().withDisableNoMotionCalibration(true);
    Pigeon2Configuration config = getPigeon2Configuration();
    config.withPigeon2Features(featureConfigs);

    return config;
  }

  public static CurrentLimitsConfigs getSafeDriveLimits() {
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 30;
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = false;
    return currentConfig;
  }

  public static CurrentLimitsConfigs getNormDriveLimits() {
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 30;

    currentConfig.StatorCurrentLimit = 140;

    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = true;
    return currentConfig;
  }
}
