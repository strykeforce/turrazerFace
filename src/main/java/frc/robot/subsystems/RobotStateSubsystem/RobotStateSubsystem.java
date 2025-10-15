package frc.robot.subsystems.RobotStateSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.laser.LaserSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotStateSubsystem {
  private TurretSubsystem turretSubsystem;
  private LaserSubsystem laserSubsystem;
  private DriveSubsystem driveSubsystem;
  // Not Focusing on the rest of robot state right now just the main functions
  public RobotStateSubsystem(
      TurretSubsystem turretSubsystem,
      LaserSubsystem laserSubsystem,
      DriveSubsystem driveSubsystem) {

    this.turretSubsystem = turretSubsystem;
    this.laserSubsystem = laserSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  private Pose2d getMotionCorrection() {
    // TODO Start Working ON  THis
    return null;
  }

  private double getTimeOfFlight() {
    return 0.0;
  }

  public enum RobotStates {
    NONE,
    TRACKING,
    SEEKING
  }
}
