package frc.robot.subsystems.RobotStateSubsystem;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.laser.LaserSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Set;
import org.strykeforce.telemetry.measurable.Measure;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class RobotStateSubsystem extends MeasurableSubsystem{
  private TurretSubsystem turretSubsystem;
  private LaserSubsystem laserSubsystem;
  private DriveSubsystem driveSubsystem;
  private Pose2d targetPos = new Pose2d();
  private AprilTagFieldLayout fieldLayout;
  private double timeOfFlight = 0.0;
  // Not Focusing on the rest of robot state right now just the main functions
  public RobotStateSubsystem(
      TurretSubsystem turretSubsystem,
      LaserSubsystem laserSubsystem,
      DriveSubsystem driveSubsystem) {

    this.turretSubsystem = turretSubsystem;
    this.laserSubsystem = laserSubsystem;
    this.driveSubsystem = driveSubsystem;
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }

  public void setTargetPos(Pose2d newTarget) {
    targetPos = newTarget;
  }

//Testing Function
  public void setTimeOfFlight(double seconds){
    timeOfFlight = seconds;
  }

  public Pose2d getTargetPos() {
    return targetPos;
  }

  private Translation2d getMotionCorrection() {

    double offsetX = driveSubsystem.getFieldRelSpeed().vxMetersPerSecond * getTimeOfFlight();
    double offsetY = driveSubsystem.getFieldRelSpeed().vyMetersPerSecond * getTimeOfFlight();

    Translation2d offset = new Translation2d(offsetX, offsetY);
    return getTargetPos().getTranslation().minus(offset);
  }

  // Function here for show because its a laser. It moves at the speed of light.
  public double getTimeOfFlight() {
    return timeOfFlight;
  }

  public void periodic(){
    turretSubsystem.pointAtPos(getTargetPos(),driveSubsystem.getPoseMeters());
  }

  public enum RobotStates {
    NONE,
    TRACKING,
    SEEKING
  }

@Override
public Set<Measure> getMeasures() {
  return Set.of();
}
}

