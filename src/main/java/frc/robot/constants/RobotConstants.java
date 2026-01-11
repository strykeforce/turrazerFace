package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.units.measure.Angle;
import org.slf4j.LoggerFactory;

public class RobotConstants {
  private org.slf4j.Logger logger = LoggerFactory.getLogger(RobotConstants.class);
  public static final String protoSerial = "032243F2";
  public static final boolean isComp = true;
  public static final int kTalonConfigTimeout = 10; // ms

  public static final double kJoystickDeadband = 0.1;
  public static final double kTriggerDeadband = 0.5;
  public static final double kTestingDeadband = 0.5;

  // Elevator
  public static Angle kElevatorFunnelSetpoint; // Elevator
  public static Angle kElevatorL1LoadSetpoint; // Elevator
  public static Angle kElevatorStowSetpoint; // Elevator

  // Biscuit
  public static TalonFXSConfiguration talonFXSConfig;
  public static MotionMagicConfigs algaeMotionConfig;
  public static MotionMagicConfigs algaeRemovalMotionConfig;
  public static MotionMagicConfigs noAlgaeMotionConfig;

  public static double kTicksPerRot;

  public static double kBiscuitZero;
  public static double kSafeToStowUpper;
  public static double kSafeToStowLower;

  // Speeds
  // public static final double kDosntHaveAlgaeSpeed = 500;

  // Setpoints
  // Idle
  public static Angle kStowSetpoint;
  public static Angle kFunnelSetpoint;
  public static Angle kL1CoralLoadSetpoint;
  public static Angle kPrestageSetpoint;
  public static Angle kPrestageAlgaeSetpoint;

  // Algae removal
  public static Angle kL2AlgaeSetpoint;
  public static Angle kL3AlgaeSetpoint;

  public static Angle kL2AlgaeRemovalSetpoint;
  public static Angle kL3AlgaeRemovalSetpoint;

  // Coral score
  public static Angle kL1CoralSetpoint;
  public static Angle kL2CoralSetpoint;
  public static Angle kL3CoralSetpoint;
  public static Angle kL4CoralSetpoint;

  // Algae obtaining
  public static Angle kFloorAlgaeSetpoint;
  public static Angle kMicAlgaeSetpoint;
  public static Angle kHpAlgaeSetpoint;

  // Algae scoring
  public static Angle kProcessorSetpoint;
  public static Angle kBargeSetpoint;
  // public static Angle kBargeBackwardSetpoint;

  public static double kTagAlignThreshold;

  public static final int kMinAutoSwitchID = 4;
  public static final int kMaxAutoSwitchID = 9;

  // TurrazerFace Constants
  public static final double kAverageScoringTime = 0;

  public RobotConstants() {
    // Proto constants
    kElevatorFunnelSetpoint = ProtoConstants.kElevatorFunnelSetpoint;
    kElevatorL1LoadSetpoint = ProtoConstants.kElevatorL1LoadSetpoint;
    kElevatorStowSetpoint = ProtoConstants.kElevatorStowSetpoint;
    talonFXSConfig = ProtoConstants.getFXSConfig();
    algaeMotionConfig = ProtoConstants.getAlgaeMotionConfig();
    algaeRemovalMotionConfig = algaeMotionConfig;
    noAlgaeMotionConfig = ProtoConstants.getNoAlgaeMotionConfig();
    kTicksPerRot = 160;
    logger.info("Using Proto Constants");

    // Biscuit

    kBiscuitZero = ProtoConstants.kZero;
    kSafeToStowUpper = ProtoConstants.kSafeToStowUpper;
    kSafeToStowLower = ProtoConstants.kSafeToStowLower;

    // Setpoints
    // Idle
    kStowSetpoint = ProtoConstants.kBiscuitStowSetpoint;
    kFunnelSetpoint = ProtoConstants.kFunnelSetpoint;
    kL1CoralLoadSetpoint = ProtoConstants.kL1CoralLoadSetpoint;
    kPrestageSetpoint = ProtoConstants.kPrestageSetpoint;
    kPrestageAlgaeSetpoint = ProtoConstants.kPrestageAlgaeSetpoint;

    // Algae removal
    kL2AlgaeSetpoint = ProtoConstants.kL2AlgaeSetpoint;
    kL3AlgaeSetpoint = ProtoConstants.kL3AlgaeSetpoint;

    kL2AlgaeRemovalSetpoint = ProtoConstants.kL2AlgaeRemovalSetpoint;
    kL3AlgaeRemovalSetpoint = ProtoConstants.kL3AlgaeRemovalSetpoint;

    // Coral score
    kL1CoralSetpoint = ProtoConstants.kL1CoralSetpoint;
    kL2CoralSetpoint = ProtoConstants.kL2CoralSetpoint;
    kL3CoralSetpoint = ProtoConstants.kL3CoralSetpoint;
    kL4CoralSetpoint = ProtoConstants.kL4CoralSetpoint;

    // Algae obtaining
    kFloorAlgaeSetpoint = ProtoConstants.kFloorAlgaeSetpoint;
    kMicAlgaeSetpoint = ProtoConstants.kMicAlgaeSetpoint;
    kHpAlgaeSetpoint = ProtoConstants.kHpAlgaeSetpoint;

    // Algae scoring
    kProcessorSetpoint = ProtoConstants.kProcessorSetpoint;
    kBargeSetpoint = ProtoConstants.kBargeSetpoint;
    // kBargeBackwardSetpoint = ProtoConstants.kBargeBackwardSetpoint;

    kTagAlignThreshold = ProtoConstants.kTagAlignThreshold;
  }

  public static class ProtoConstants {

    // Biscuit
    public static TalonFXSConfiguration talonFXSConfig;
    public static MotionMagicConfigs alageMotionConfig;
    public static MotionMagicConfigs noAlageMotionConfig;

    public static double kTicksPerRot = 160;

    public static final double kZero = .69;
    public static final double kSafeToStowUpper = 4.06;
    public static final double kSafeToStowLower = -5 / 2;

    // Speeds
    // public static final double kDosntHaveAlgaeSpeed = 500;

    // Setpoints
    // Idle
    public static Angle kBiscuitStowSetpoint = Rotations.of(1.862 / 2);
    public static Angle kFunnelSetpoint = kBiscuitStowSetpoint;
    public static Angle kL1CoralLoadSetpoint = Rotations.of(3.96);
    public static Angle kPrestageSetpoint = kBiscuitStowSetpoint;
    public static Angle kPrestageAlgaeSetpoint = Rotations.of(9.089 / 2);

    // Algae removal
    public static Angle kL2AlgaeSetpoint = Rotations.of(24.104 / 2);
    public static Angle kL3AlgaeSetpoint = Rotations.of(24.562 / 2);

    public static Angle kL2AlgaeRemovalSetpoint = kBiscuitStowSetpoint;
    public static Angle kL3AlgaeRemovalSetpoint = kBiscuitStowSetpoint;

    public static double kTagAlignThreshold = 20.0 / 2;

    // Coral score
    public static Angle kL1CoralSetpoint = Rotations.of(12.7);
    public static Angle kL2CoralSetpoint = kBiscuitStowSetpoint;
    public static Angle kL3CoralSetpoint = kBiscuitStowSetpoint;
    public static Angle kL4CoralSetpoint = kBiscuitStowSetpoint;

    // Algae obtaining
    public static Angle kFloorAlgaeSetpoint = Rotations.of(49.627 / 2);
    public static Angle kMicAlgaeSetpoint = Rotations.of(51.61872 / 2);
    public static Angle kHpAlgaeSetpoint = Rotations.of(16.97559 / 2);

    // Algae scoring
    public static Angle kProcessorSetpoint = Rotations.of(41.193 / 2);
    public static Angle kBargeSetpoint = Rotations.of(12.3489 / 2);
    // public static Angle kBargeBackwardSetpoint = Rotations.of(-12.3489); // 9.089

    // Elevator
    public static Angle kElevatorFunnelSetpoint = Rotations.of(2.03125);
    public static Angle kElevatorL1LoadSetpoint = Rotations.of(1.76);
    public static Angle kElevatorStowSetpoint = kElevatorFunnelSetpoint;
    public static Angle kMaxFwd = kMicAlgaeSetpoint.plus(Rotations.of(5));
    public static Angle kMaxRev = kPrestageSetpoint.minus(Rotations.of(5));

    public static MotionMagicConfigs getAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(300)
              .withMotionMagicCruiseVelocity(50)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(600);
      return algaeConfig;
    }

    public static MotionMagicConfigs getAlgaeRemovalMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(300)
              .withMotionMagicCruiseVelocity(30)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(1500);
      return algaeConfig;
    }

    public static MotionMagicConfigs getNoAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(300)
              .withMotionMagicCruiseVelocity(80)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(1800);
      return algaeConfig;
    }

    public static TalonFXSConfiguration getFXSConfig() {
      TalonFXSConfiguration fxsConfig = new TalonFXSConfiguration();

      CurrentLimitsConfigs current =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(0)
              .withStatorCurrentLimitEnable(false)
              .withSupplyCurrentLimit(20)
              .withSupplyCurrentLowerLimit(5)
              .withSupplyCurrentLowerTime(2)
              .withSupplyCurrentLimitEnable(true);
      fxsConfig.CurrentLimits = current;

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
      fxsConfig.HardwareLimitSwitch = hwLimit;

      SoftwareLimitSwitchConfigs swLimit =
          new SoftwareLimitSwitchConfigs()
              .withForwardSoftLimitEnable(true)
              .withForwardSoftLimitThreshold(kMaxFwd)
              .withReverseSoftLimitEnable(true)
              .withReverseSoftLimitThreshold(kMaxRev);
      fxsConfig.SoftwareLimitSwitch = swLimit;

      Slot0Configs slot0 =
          new Slot0Configs()
              .withKP(2)
              .withKI(0)
              .withKD(0)
              .withGravityType(GravityTypeValue.Elevator_Static)
              .withKG(0)
              .withKS(0)
              .withKV(0.1)
              .withKA(0);
      fxsConfig.Slot0 = slot0;

      fxsConfig.MotionMagic = getNoAlgaeMotionConfig();

      MotorOutputConfigs motorOut =
          new MotorOutputConfigs()
              .withDutyCycleNeutralDeadband(0.01)
              .withNeutralMode(NeutralModeValue.Brake);
      fxsConfig.MotorOutput = motorOut;

      CommutationConfigs commutation =
          new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST);
      fxsConfig.Commutation = commutation;

      ExternalFeedbackConfigs feedBack =
          new ExternalFeedbackConfigs()
              .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Commutation);
      fxsConfig.ExternalFeedback = feedBack;

      return fxsConfig;
    }
  }
}
