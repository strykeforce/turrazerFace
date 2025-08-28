package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.gyro.SF_AHRS;
import org.strykeforce.gyro.SF_PIGEON2;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.swerve.FXSwerveModule;
import org.strykeforce.swerve.OdometryStrategy;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule.ClosedLoopUnits;
import org.strykeforce.telemetry.TelemetryService;

public class SwerveFXS implements SwerveIO, Checkable {
  @HealthCheck private final SwerveDrive swerveDrive;

  // Grapher stuff
  private PoseEstimatorOdometryStrategy odometryStrategy;

  private SF_PIGEON2 pigeon;
  private SF_AHRS navx;
  private Rotation2d navxOffset = new Rotation2d();

  private TalonFXSConfigurator configuratorFXS;
  private TalonFXConfigurator configurator;

  private TalonFXS[] azimuths = new TalonFXS[4];
  private TalonFX[] drives = new TalonFX[4];

  private FXSwerveModule[] swerveModules;
  private SwerveDriveKinematics kinematics;
  private double fieldY = 0.0;
  private double fieldX = 0.0;
  private boolean didZero = false;

  public SwerveFXS() {

    var moduleBuilder =
        new FXSwerveModule.FXBuilder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond)
            .latencyCompensation(true)
            .encoderOpposed(false);
    swerveModules = new FXSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();
    didZero = true;
    for (int i = 0; i < 4; i++) {
      var azimuthFXS = new TalonFXS(i, "*");
      configuratorFXS = azimuthFXS.getConfigurator();
      configuratorFXS.apply(new TalonFXSConfiguration()); // factory default
      configuratorFXS.apply(DriveConstants.getAzimuthFXSConfig());

      azimuthFXS.getRawPulseWidthPosition().setUpdateFrequency(200);

      azimuths[i] = azimuthFXS;

      var driveTalon = new TalonFX(i + 10, "*");
      configurator = driveTalon.getConfigurator();
      configurator.apply(new TalonFXConfiguration()); // factory default
      configurator.apply(DriveConstants.getDriveTalonConfig());
      drives[i] = driveTalon;

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthFXS)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .closedLoopUnits(ClosedLoopUnits.VOLTAGE)
              .build();
      boolean zeroCheck = swerveModules[i].zeroAndCheck();
      didZero = zeroCheck && didZero;
    }

    pigeon = new SF_PIGEON2(DriveConstants.kPigeonCanID, "*");
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

  @Override
  public void zeroModules() {
    didZero = true;
    for (int i = 0; i < 4; i++) {
      boolean zeroCheck = swerveModules[i].zeroAndCheck();
      didZero = zeroCheck && didZero;
    }
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

  public double getAvgDriveCurrent() {
    double sum = 0;

    for (int i = 0; i < 4; i++) {
      sum += drives[i].getStatorCurrent().getValueAsDouble();
    }

    return sum / 4.0;
  }

  public double getFrontDriveAvgVel() {
    double sum = 0;

    for (int i = 0; i < 2; i++) {
      sum += FastMath.abs(drives[i].getVelocity().getValueAsDouble());
    }

    return sum / 2.0;
  }

  public double getRearDriveAvgVel() {
    double sum = 0;

    for (int i = 2; i < 4; i++) {
      sum += FastMath.abs(drives[i].getVelocity().getValueAsDouble());
    }

    return sum / 2.0;
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

  public void disableNoMotionCal() {
    pigeon.applyConfig(DriveConstants.getPigeon2NoMotionDisabledConfiguration());
  }

  @Override
  public void resetGyro() {
    swerveDrive.resetGyro();
    navx.reset();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    navx.reset();
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
      azimuths[i].set(vel);
    }
  }

  @Override
  public void configDriveCurrents(CurrentLimitsConfigs config) {
    for (int i = 0; i < 4; i++) {
      drives[i].getConfigurator().apply(config);
    }
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
    for (int i = 0; i < 4; ++i) {
      inputs.azimuthVels[i] = azimuths[i].getVelocity().getValueAsDouble();
      inputs.azimuthCurrent[i] = azimuths[i].getSupplyCurrent().getValueAsDouble();
    }
    inputs.avgDriveCurrent = getAvgDriveCurrent();
    inputs.avgRearDriveVel = getRearDriveAvgVel();
    inputs.avgFrontDriveVel = getFrontDriveAvgVel();
    inputs.robotRelSpeed = getRobotRelSpeed();
    inputs.fieldRelSpeed = getFieldRelSpeed(inputs.robotRelSpeed);
    inputs.fieldY = fieldY;
    inputs.fieldX = fieldX;
    inputs.didZero = didZero;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    swerveDrive.registerWith(telemetryService);
    pigeon.registerWith(telemetryService);
  }
}
