package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import java.util.Set;
import java.util.function.BooleanSupplier;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {
  private static final org.slf4j.Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private final SwerveIO io;
  private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
  private final HolonomicDriveController holonomicController;

  private final ProfiledPIDController omegaController;
  private final PIDController xController;
  private final PIDController yController;

  public DriveStates currDriveState = DriveStates.IDLE;
  private ChassisSpeeds holoContOutput = new ChassisSpeeds();

  private double trajectoryActive = 0.0;

  private double driveMultiplier = 1.0;

  private int gyroDifferentCount = 0;
  private int gyroCorrectionCount = 0;

  private boolean ignoreSticks = false;

  private RobotStateSubsystem robotStateSubsystem;

  public DriveSubsystem(SwerveIO io) {
    this.io = io;
    // Setup Holonomic Controller
    omegaController =
        new ProfiledPIDController(
            DriveConstants.kPOmega,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxAutoOmega, DriveConstants.kMaxAccelOmega));
    omegaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    xController =
        new PIDController(
            DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    // xController.setIntegratorRange(DriveConstants.kIMin, DriveConstants.kIMax);
    yController =
        new PIDController(
            DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    // yController.setIntegratorRange(DriveConstants.kIMin, DriveConstants.kIMax);
    holonomicController = new HolonomicDriveController(xController, yController, omegaController);
    // Disabling the holonomic controller makes the robot directly follow the
    // trajectory output (no
    // closing the loop on x,y,theta errors)
    holonomicController.setEnabled(true);

    setAutoDebugMsg("Nothing");
  }

  public boolean hasZeroed() {
    return inputs.didZero;
  }

  public void zeroModules() {
    io.zeroModules();
  }

  // Open-Loop Swerve Movements
  public void drive(double vXmps, double vYmps, double vOmegaRadps) {
    if (!ignoreSticks) {
      io.drive(vXmps * driveMultiplier, vYmps * driveMultiplier, vOmegaRadps, true);
    }
  }

  public void stopDriving() {
    this.move(0, 0, 0, false);
    io.drive(0, 0, 0, false);
  }

  public void setAzimuthVel(double vel) {
    io.setAzimuthVel(vel);
  }

  // Closed-Loop (Velocity Controlled) Swerve Movement
  public void move(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/Move X", vXmps);
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/Move Y", vYmps);
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/Move Omega", vOmegaRadps);

    io.move(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  public double getHolonomicControllerOmegaErrorRadians() {
    return omegaController.getPositionError();
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
    logger.info("reset odometry with: {}", pose);
  }

  public void setAutoDebugMsg(String msg) {
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/Auto Drive Info", msg);
  }

  public void setIgnoreSticks(boolean ignore) {
    org.littletonrobotics.junction.Logger.recordOutput("DriveSubsystem/Ignoring Sticks", ignore);
    this.ignoreSticks = ignore;
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    org.littletonrobotics.junction.Logger.recordOutput("DriveSubsystem/Pose from vision", pose);
    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Time Since Vision Update",
        RobotController.getFPGATime() / 1_000_000 - timestamp);
    io.addVisionMeasurement(pose, timestamp);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevvs) {
    org.littletonrobotics.junction.Logger.recordOutput("DriveSubsystem/Pose from vision", pose);
    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/stdDevvs", stdDevvs.get(0, 0));
    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Time Since Vision Update",
        RobotController.getFPGATime() / 1_000_000.0 - timestamp);

    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/FPGA Seconds", RobotController.getFPGATime() / 1_000_000.0);
    org.littletonrobotics.junction.Logger.recordOutput("DriveSubsystem/Result Seconds", timestamp);

    io.addVisionMeasurement(pose, timestamp, stdDevvs);
  }

  public void resetHolonomicController(double yaw) {
    xController.reset();
    yController.reset();
    omegaController.reset(yaw);
    holonomicController.getThetaController().reset(yaw);
    logger.info("Holonomic Controller Reset: {}", yaw);
  }

  public void resetHolonomicController() {
    xController.reset();
    yController.reset();
    omegaController.reset(inputs.gyroRotation2d.getRadians());
    holonomicController.getThetaController().reset(inputs.gyroRotation2d.getRadians());
    logger.info("Holonomic Controller Reset: {}", inputs.gyroRotation2d.getRadians());
  }

  public void resetOmegaController() {
    omegaController.reset(inputs.gyroRotation2d.getRadians());
  }

  // Getters/Setters
  public Pose2d getPoseMeters() {
    return inputs.poseMeters;
  }

  public Rotation2d getGyroRotation2d() {
    return inputs.gyroRotation2d;
  }

  public ChassisSpeeds getFieldRelSpeed() {
    return inputs.fieldRelSpeed;
  }

  public ChassisSpeeds getRobotRelSpeed() {
    return inputs.robotRelSpeed;
  }

  public void setDriveState(DriveStates state) {
    currDriveState = state;
  }

  public BooleanSupplier getAzimuth1FwdLimitSupplier() {
    return io.getAzimuth1FwdLimitSwitch();
  }

  public boolean isDriveStill() {
    ChassisSpeeds cs = inputs.fieldRelSpeed;
    double vX = cs.vxMetersPerSecond;
    double vY = cs.vyMetersPerSecond;

    // Take fieldRel Speed and get the magnitude of the vector
    double wheelSpeed = Math.sqrt(FastMath.pow2(vX) + FastMath.pow2(vY));

    boolean velStill = Math.abs(wheelSpeed) <= DriveConstants.kSpeedStillThreshold;
    boolean gyroStill = isGyroStill();

    return velStill && gyroStill;
  }

  public boolean isGyroStill() {
    return Math.abs(inputs.gyroRate) <= DriveConstants.kGyroRateStillThreshold;
  }

  public void setGyroOffset(Rotation2d rotation) {
    io.setBothGyroOffset(apply(rotation));
    Logger.recordOutput("DriveSubsystem/gyroOffset", rotation);
  }

  public void setEnableHolo(boolean enabled) {
    holonomicController.setEnabled(enabled);
    logger.info("Holonomic Controller Enabled: {}", enabled);
  }

  public void prepClimb() {
    io.setDriveCoast(true);
    io.setSwerveModuleAngles(
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90));
  }

  public void teleResetGyro() {
    setAutoDebugMsg("Reset Gyro");
    logger.info("Driver Joystick: Reset Gyro");
    double gyroResetDegs = robotStateSubsystem.getAllianceColor() == Alliance.Blue ? 0.0 : 180.0;
    io.setBothGyroOffset(Rotation2d.fromDegrees(gyroResetDegs));
    Logger.recordOutput("DriveSubsystem/gyroOffset", Rotation2d.fromDegrees(gyroResetDegs));
    io.resetGyro();
    io.resetOdometry(
        new Pose2d(inputs.poseMeters.getTranslation(), Rotation2d.fromDegrees(gyroResetDegs)));
  }

  // Make whether a trajectory is currently active obvious on grapher
  public void grapherTrajectoryActive(boolean active) {
    if (active) trajectoryActive = 1.0;
    else trajectoryActive = 0.0;
  }

  // Field flipping stuff
  public boolean shouldFlip() {
    return robotStateSubsystem.getAllianceColor() == Alliance.Red;
  }

  public Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(DriveConstants.kFieldMaxX - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  public Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          DriveConstants.kFieldMaxX - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  public Rotation2d apply(Rotation2d rotation) {
    logger.info(
        "initial target yaw: {}, cos: {}, sin: {}", rotation, rotation.getCos(), rotation.getSin());
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else return rotation;
  }

  public double apply(double x) {
    logger.info("initial x: {}", x);
    if (shouldFlip()) {
      return DriveConstants.kFieldMaxX - x;
    } else return x;
  }

  // Control Methods
  public void lockZero() {
    Rotation2d rot = Rotation2d.fromDegrees(0.0);
    io.setSwerveModuleAngles(rot, rot, rot, rot);
  }

  public void toSafeHold() {
    setDriveState(DriveStates.SAFE_HOLD);
    io.configDriveCurrents(DriveConstants.getSafeDriveLimits());
  }

  public void toIdle() {
    setDriveState(DriveStates.IDLE);
    io.configDriveCurrents(DriveConstants.getNormDriveLimits());
  }

  public PIDController getxController() {
    return xController;
  }

  public PIDController getyController() {
    return yController;
  }

  public ProfiledPIDController getomegaController() {
    return omegaController;
  }

  public PIDController getomegaControllerNonProfiled() {
    return new PIDController(
        omegaController.getP(), omegaController.getI(), omegaController.getD());
  }

  public double getAvgDriveCurrent() {
    return FastMath.abs(inputs.avgDriveCurrent);
  }

  public double getAvgRearDriveVel() {
    return inputs.avgRearDriveVel;
  }

  public void setDriveMultiplier(double multiplier) {
    driveMultiplier = multiplier;
  }

  public double getDriveMultiplier() {
    return driveMultiplier;
  }

  public void removeDriveMultiplier() {
    driveMultiplier = 1.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Swerve Pose", inputs.swervePose);

    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Gyro Correction Count", gyroCorrectionCount);

    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Gyro Disagreement",
        inputs.gyroRotation2d.minus(inputs.navxRotation2d).getDegrees());

    if (Math.abs(inputs.gyroRotation2d.minus(inputs.navxRotation2d).getDegrees())
        > DriveConstants.kGyroDifferentThreshold) {
      gyroDifferentCount++;
    } else {
      gyroDifferentCount = 0;
    }
    if (gyroDifferentCount > DriveConstants.kGyroDifferentCount && isDriveStill()) {
      io.setPigeonGyroOffset(
          inputs.navxRotation2d.minus(inputs.gyroRotation2d).plus(io.getPigeonGyroOffset()));
      logger.info(
          "NavX gyro correction degs {} -> {}",
          inputs.gyroRotation2d.getDegrees(),
          inputs.navxRotation2d.getDegrees());
      gyroDifferentCount = 0;
      gyroCorrectionCount++;
    }

    switch (currDriveState) {
      case IDLE -> {}
      case SAFE -> {}
      case SAFE_HOLD -> {}
      default -> {}
    }
  }

  public enum DriveStates {
    IDLE,
    SAFE,
    SAFE_HOLD
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    io.registerWith(telemetryService);
    super.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("State", () -> currDriveState.ordinal()),
        new Measure("Gyro roll", () -> inputs.gyroRoll),
        new Measure("Gyro pitch", () -> inputs.gyroPitch),
        new Measure("Gyro Rotation2D(deg)", () -> inputs.gyroRotation2d.getDegrees()),
        new Measure("Odometry X", () -> inputs.poseMeters.getX()),
        new Measure("Odometry Y", () -> inputs.poseMeters.getY()),
        new Measure("Odometry Rotation2D(deg)", () -> inputs.poseMeters.getRotation().getDegrees()),
        new Measure("Holonomic Cont Vx", () -> holoContOutput.vxMetersPerSecond),
        new Measure("Holonomic Cont Vy", () -> holoContOutput.vyMetersPerSecond),
        new Measure("Holonomic Cont Vomega", () -> holoContOutput.omegaRadiansPerSecond),
        new Measure(
            "Holo Controller Omega Err",
            () -> holonomicController.getThetaController().getPositionError()),
        new Measure(
            "Holo Controller Omega Setpoint",
            () -> holonomicController.getThetaController().getSetpoint().position),
        new Measure("Trajectory Active", () -> trajectoryActive));
  }
}
