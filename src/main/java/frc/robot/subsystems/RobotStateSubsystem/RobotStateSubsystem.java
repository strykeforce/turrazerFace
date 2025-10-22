package frc.robot.subsystems.RobotStateSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.laser.LaserSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotStateSubsystem {
  private TurretSubsystem turretSubsystem;
  private LaserSubsystem laserSubsystem;
  private DriveSubsystem driveSubsystem;
  private Translation2d targetPos = new Translation2d();
  // Not Focusing on the rest of robot state right now just the main functions
  public RobotStateSubsystem(
      TurretSubsystem turretSubsystem,
      LaserSubsystem laserSubsystem,
      DriveSubsystem driveSubsystem) {

    this.turretSubsystem = turretSubsystem;
    this.laserSubsystem = laserSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  public void setTargetPos(Translation2d newTarget){
    targetPos = newTarget;
  }
  public Translation2d getTargetPos(){
    return targetPos;
  }
  private Translation2d getMotionCorrection() {

    double offsetX = driveSubsystem.getFieldRelSpeed().vxMetersPerSecond 
    * getTimeOfFlight();
    double offsetY = driveSubsystem.getFieldRelSpeed().vyMetersPerSecond
    * getTimeOfFlight();

    Translation2d offset = new Translation2d(offsetX, offsetY);
    return getTargetPos().minus(offset);
  }

  //Function here for show because its a laser. It moves at the speed of light.
  private double getTimeOfFlight() {
    return 0.0;
  }

  public enum RobotStates {
    NONE,
    TRACKING,
    SEEKING
  }
}
