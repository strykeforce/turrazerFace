package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.swerve.OdometryStrategy;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.telemetry.TelemetryService;

public interface SwerveIO {

  @AutoLog
  public static class SwerveIOInputs {
    public double odometryX = 0.0;
    public double odometryY = 0.0;
    public Pose2d swervePose;
    public double odometryRotation2D = 0.0;
    public Rotation2d gyroRotation2d = new Rotation2d();
    public Rotation2d navxRotation2d = new Rotation2d();
    public double normalizedGyroRotation = 0.0;
    public double gyroPitch = 0.0;
    public double gyroRoll = 0.0;
    public double gyroRate = 0.0;
    public boolean isConnected = false;
    public Pose2d poseMeters = new Pose2d();
    public double pigeonTemp = 0;
    public double fieldX = 0;
    public double fieldY = 0;
    public ChassisSpeeds robotRelSpeed = new ChassisSpeeds();
    public ChassisSpeeds fieldRelSpeed = new ChassisSpeeds();
    public double[] azimuthVels = {0, 0, 0, 0};
    public double[] azimuthCurrent = {0, 0, 0, 0};
    public double avgDriveCurrent = 0;
    public double avgRearDriveVel = 0;
    public double avgFrontDriveVel = 0;
    public boolean didZero = false;
  }

  private SwerveModule[] getSwerveModules() {
    return null;
  }

  public default void setSwerveModuleAngles(
      Rotation2d FL, Rotation2d FR, Rotation2d BL, Rotation2d BR) {}

  private SwerveModulePosition[] getSwerveModulePositions() {
    return null;
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    return null;
  }

  private ChassisSpeeds getRobotRelSpeed() {
    return null;
  }

  public default SwerveDriveKinematics getKinematics() {
    return null;
  }

  public default Rotation2d getPigeonGyroOffset() {
    return null;
  }

  public default void setDriveCoast(boolean coast) {}

  public default void setOdometry(OdometryStrategy Odom) {}

  public default void setPigeonGyroOffset(Rotation2d rotation) {}

  public default void setBothGyroOffset(Rotation2d rotation) {}

  public default void resetGyro() {}

  public default void updateSwerve() {}

  public default void resetOdometry(Pose2d pose) {}

  public default void addVisionMeasurement(Pose2d pose, double timestamp) {}

  public default void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

  public default void drive(
      double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {}

  public default void move(
      double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {}

  public default void setAzimuthVel(double vel) {}

  public default void updateInputs(SwerveIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void configDriveCurrents(CurrentLimitsConfigs config) {}

  public default BooleanSupplier getAzimuth1FwdLimitSwitch() {
    return () -> false;
  }

  public default void zeroModules() {}

  public default void prepForAuto(Pose2d pose2d, double offsetDegrees, Alliance alliance) {}
}
