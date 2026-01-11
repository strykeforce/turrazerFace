package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.gyro.SF_AHRS;
import org.strykeforce.gyro.SF_PIGEON2;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.swerve.OdometryStrategy;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule.ClosedLoopUnits;
import org.strykeforce.telemetry.TelemetryService;

public class Swerve implements SwerveIO, Checkable {
  private final SwerveDrive swerveDrive;

  // Grapher stuff
  private PoseEstimatorOdometryStrategy odometryStrategy;

  private SF_PIGEON2 pigeon;
  private SF_AHRS navx;
  private Rotation2d navxOffset = new Rotation2d();

  private TalonFXConfigurator configurator;

  private TalonSRX[] azimuths = new TalonSRX[4];
  private TalonFX[] drives = new TalonFX[4];

  private V6TalonSwerveModule[] swerveModules;
  private SwerveDriveKinematics kinematics;
  private double fieldY = 0.0;
  private double fieldX = 0.0;

  private StatusSignal<Current> drive10StatorCurrent;
  private StatusSignal<Current> drive11StatorCurrent;
  private StatusSignal<Current> drive12StatorCurrent;
  private StatusSignal<Current> drive13StatorCurrent;
  private StatusSignal<AngularVelocity> drive10Velocity;
  private StatusSignal<AngularVelocity> drive11Velocity;
  private StatusSignal<AngularVelocity> drive12Velocity;
  private StatusSignal<AngularVelocity> drive13Velocity;
  private boolean didZero = false;

  public Swerve() {

    var moduleBuilder =
        new V6TalonSwerveModule.V6Builder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond)
            .latencyCompensation(true);

    swerveModules = new V6TalonSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuths[i] = azimuthTalon;
      azimuthTalon.configFactoryDefault(RobotConstants.kTalonConfigTimeout);
      azimuthTalon.configAllSettings(
          DriveConstants.getAzimuthTalonConfig(), RobotConstants.kTalonConfigTimeout);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      var driveTalon = new TalonFX(i + 10);
      drives[i] = driveTalon;
      configurator = driveTalon.getConfigurator();
      configurator.apply(new TalonFXConfiguration()); // factory default
      configurator.apply(DriveConstants.getDriveTalonConfig());
      driveTalon.getSupplyVoltage().setUpdateFrequency(100);
      driveTalon.getSupplyCurrent().setUpdateFrequency(100);
      driveTalon.getClosedLoopReference().setUpdateFrequency(200);

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .closedLoopUnits(ClosedLoopUnits.VOLTAGE)
              .build();
      swerveModules[i].loadAndSetAzimuthZeroReference();
    }

    drive10StatorCurrent = drives[0].getStatorCurrent();
    drive11StatorCurrent = drives[1].getStatorCurrent();
    drive12StatorCurrent = drives[2].getStatorCurrent();
    drive13StatorCurrent = drives[3].getStatorCurrent();
    drive10Velocity = drives[0].getVelocity();
    drive11Velocity = drives[1].getVelocity();
    drive12Velocity = drives[2].getVelocity();
    drive13Velocity = drives[3].getVelocity();
    didZero = true;

    pigeon = new SF_PIGEON2(DriveConstants.kPigeonCanID, "rio");
    pigeon.applyConfig(DriveConstants.getPigeon2Configuration());
    navx = new SF_AHRS();
    swerveDrive = new SwerveDrive(false, 0.02, pigeon, swerveModules);
    swerveDrive.resetGyro();
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));

    kinematics = swerveDrive.getKinematics();

    odometryStrategy =
        new PoseEstimatorOdometryStrategy(
            swerveDrive.getHeading(),
            new Pose2d(),
            swerveDrive.getKinematics(),
            VisionConstants.kStateStdDevs,
            VisionConstants.kLocalMeasurementStdDevs,
            VisionConstants.kVisionMeasurementStdDevs,
            getSwerveModulePositions());

    swerveDrive.setOdometry(odometryStrategy);
  }

  // Getters/Setter
  @Override
  public String getName() {
    return "Swerve";
  }

  private SwerveModule[] getSwerveModules() {
    return swerveDrive.getSwerveModules();
  }

  public void setSwerveModuleAngles(Rotation2d FL, Rotation2d FR, Rotation2d BL, Rotation2d BR) {
    swerveModules[0].setAzimuthRotation2d(FL);
    swerveModules[1].setAzimuthRotation2d(FR);
    swerveModules[2].setAzimuthRotation2d(BL);
    swerveModules[3].setAzimuthRotation2d(BR);
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModule[] swerveModules = getSwerveModules();
    SwerveModulePosition[] temp = {null, null, null, null};
    for (int i = 0; i < 4; ++i) {
      temp[i] = swerveModules[i].getPosition();
    }
    return temp;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    V6TalonSwerveModule[] swerveModules = (V6TalonSwerveModule[]) swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
    return swerveModuleStates;
  }

  private ChassisSpeeds getRobotRelSpeed() {
    SwerveDriveKinematics kinematics = swerveDrive.getKinematics();
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; ++i) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
    return kinematics.toChassisSpeeds(swerveModuleStates);
  }

  private ChassisSpeeds getFieldRelSpeed(ChassisSpeeds roboRelSpeed) {
    Rotation2d heading = swerveDrive.getHeading().unaryMinus();
    fieldX =
        roboRelSpeed.vxMetersPerSecond * heading.getCos()
            + roboRelSpeed.vyMetersPerSecond * heading.getSin();
    fieldY =
        -roboRelSpeed.vxMetersPerSecond * heading.getSin()
            + roboRelSpeed.vyMetersPerSecond * heading.getCos();

    return new ChassisSpeeds(fieldX, fieldY, roboRelSpeed.omegaRadiansPerSecond);
  }

  public Rotation2d getPigeonGyroOffset() {
    return swerveDrive.getGyroOffset();
  }

  @Override
  public void setOdometry(OdometryStrategy odom) {
    swerveDrive.setOdometry(odom);
  }

  @Override
  public void setPigeonGyroOffset(Rotation2d rotation) {
    swerveDrive.setGyroOffset(rotation);
  }

  @Override
  public void setBothGyroOffset(Rotation2d rotation) {
    swerveDrive.setGyroOffset(rotation);
    navxOffset = rotation;
  }

  @Override
  public void setDriveCoast(boolean coast) {
    for (int i = 0; i < 4; i++) {
      drives[i]
          .getConfigurator()
          .apply(
              DriveConstants.getDriveTalonConfig()
                  .MotorOutput
                  .withNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake));
    }
  }

  public void disableNoMotionCal() {
    pigeon.applyConfig(DriveConstants.getPigeon2NoMotionDisabledConfiguration());
  }

  @Override
  public void resetGyro() {
    swerveDrive.resetGyro();
    navx.reset();
  }

  @Override
  public void prepForAuto(Pose2d pose2d, double offsetDegrees, Alliance alliance) {
    double gyroResetDegs = alliance == Alliance.Blue ? 0.0 : 180.0;
    setBothGyroOffset(Rotation2d.fromDegrees(gyroResetDegs));
    resetGyro();
    setBothGyroOffset(Rotation2d.fromDegrees(offsetDegrees));
    resetOdometry(pose2d);
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    // navx.reset();
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    Logger.recordOutput("Drive/VisionSampledPose", odometryStrategy.getSample(timestamp));
    odometryStrategy.addVisionMeasurement(pose, timestamp);
  }

  @Override
  public void addVisionMeasurement(Pose2d pose2d, double timestamp, Matrix<N3, N1> stdDevs) {
    Logger.recordOutput("Drive/VisionSampledPose", odometryStrategy.getSample(timestamp));
    odometryStrategy.addVisionMeasurement(pose2d, timestamp, stdDevs);
  }

  @Override
  public void drive(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    swerveDrive.drive(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  @Override
  public void move(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    swerveDrive.move(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  @Override
  public void setAzimuthVel(double vel) {
    for (int i = 0; i < 4; i++) {
      azimuths[i].set(TalonSRXControlMode.PercentOutput, vel);
    }
  }

  @Override
  public void configDriveCurrents(CurrentLimitsConfigs config) {
    for (int i = 0; i < 4; i++) {
      drives[i].getConfigurator().apply(config);
    }
  }

  public double getAvgDriveCurrent() {
    double sum = 0;

    for (int i = 0; i < 4; i++) {
      sum += drives[i].getStatorCurrent().getValueAsDouble();
    }

    return sum / 4.0;
  }

  public double getRearDriveAvgVel() {
    double sum = 0;

    for (int i = 2; i < 4; i++) {
      sum += FastMath.abs(drives[i].getVelocity().getValueAsDouble());
    }

    return sum / 2.0;
  }

  @Override
  public void zeroModules() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].loadAndSetAzimuthZeroReference();
    }
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    swerveDrive.updateInputs(); // Call before swerveDrive.periodic()
    swerveDrive.periodic();

    inputs.odometryX = swerveDrive.getPoseMeters().getX();
    inputs.odometryY = swerveDrive.getPoseMeters().getY();
    inputs.swervePose = odometryStrategy.getPoseMeters();
    inputs.odometryRotation2D = swerveDrive.getPoseMeters().getRotation().getDegrees();
    inputs.gyroRotation2d = swerveDrive.getHeading();
    inputs.navxRotation2d = navx.getRotation2d().rotateBy(navxOffset);
    inputs.normalizedGyroRotation =
        FastMath.normalizeMinusPiPi(swerveDrive.getHeading().getRadians());
    inputs.gyroPitch = pigeon.getPitch();
    inputs.gyroRoll = pigeon.getRoll();
    inputs.gyroRate = swerveDrive.getGyroRate();
    inputs.isConnected = pigeon.getPigeon2().getUpTime().hasUpdated();
    inputs.poseMeters = swerveDrive.getPoseMeters();
    inputs.pigeonTemp = pigeon.getPigeon2().getTemperature().getValueAsDouble();
    inputs.robotRelSpeed = getRobotRelSpeed();
    inputs.fieldRelSpeed = getFieldRelSpeed(inputs.robotRelSpeed);
    inputs.fieldY = fieldY;
    inputs.fieldX = fieldX;
    inputs.didZero = didZero;

    for (int i = 0; i < 4; ++i) {
      inputs.azimuthVels[i] = azimuths[i].getSelectedSensorVelocity();
      inputs.azimuthCurrent[i] = azimuths[i].getSupplyCurrent();
    }
    BaseStatusSignal.refreshAll(
        drive10StatorCurrent,
        drive11StatorCurrent,
        drive12StatorCurrent,
        drive13StatorCurrent,
        drive10Velocity,
        drive11Velocity,
        drive12Velocity,
        drive13Velocity);
    // inputs.avgDriveCurrent = getAvgDriveCurrent();
    // inputs.avgRearDriveVel = getRearDriveAvgVel();

    inputs.avgDriveCurrent =
        (drive10StatorCurrent.getValueAsDouble()
                + drive11StatorCurrent.getValueAsDouble()
                + drive12StatorCurrent.getValueAsDouble()
                + drive13StatorCurrent.getValueAsDouble())
            / 4;
    inputs.avgRearDriveVel =
        (Math.abs(drive12Velocity.getValueAsDouble())
                + Math.abs(drive13Velocity.getValueAsDouble()))
            / 2;
    inputs.avgRearDriveVel =
        (Math.abs(drive10Velocity.getValueAsDouble())
                + Math.abs(drive11Velocity.getValueAsDouble()))
            / 2;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    swerveDrive.registerWith(telemetryService);
    pigeon.registerWith(telemetryService);
  }
}
