package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import WallEye.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.turret.TurretSubsystemIOFX;
import java.util.ArrayList;
import java.util.Set;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {

  // Array of cameras
  private WallEyeCam[] cams;

  // Array of camera positions
  private Translation3d[] camPositions = {
    VisionConstants.kCam1Pose.getTranslation(),
    VisionConstants.kCam2Pose.getTranslation(),
    VisionConstants.kCam3Pose.getTranslation(),
  };

  private double[] camHeights = {
    camPositions[0].getZ(), camPositions[1].getZ(), camPositions[2].getZ(),
  };

  // Array of camera rotations
  private Rotation3d[] camRotations = {
    VisionConstants.kCam1Pose.getRotation(),
    VisionConstants.kCam2Pose.getRotation(),
    VisionConstants.kCam3Pose.getRotation(),
  };

  // Array of camera names
  private String[] camNames = {
    VisionConstants.kCam1Name, VisionConstants.kCam2Name, VisionConstants.kCam3Name,
  };

  private int trustedCameraYawIdx = -1;

  // private DriveSubsystem driveSubsystem;
  /*Because we use two seperate loggers we can import one and then define the
  other here.*/
  private org.slf4j.Logger textLogger;
  private UdpSubscriber[] udpSubscriber;
  private AprilTagFieldLayout field;
  private boolean visionUpdating = true;
  private int minTags;
  private CircularBuffer<Double> gyroBuffer =
      new CircularBuffer<Double>(VisionConstants.kCircularBufferSize);
  private double timeSinceLastUpdate;
  private int updatesToWheels;
  private Matrix adaptiveMatrix;
  private ArrayList<Pair<WallEyeResult, Integer>> validResults = new ArrayList<>();
  private WallEyeTagResult[] lastResult = new WallEyeTagResult[VisionConstants.kNumCams];
  private Matrix<N3, N1> adativeMatrix;
  private Matrix<N3, N1> stdMatrix;
  private TurretSubsystemIOFX turretSubsystem;
  private boolean[] acceptUpdates = new boolean[VisionConstants.kNumCams];
  private boolean ignoreRearCams = false;
  private boolean isAuto = false;

  public VisionSubsystem(TurretSubsystemIOFX turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    // this.driveSubsystem = driveSubsystem;
    textLogger = LoggerFactory.getLogger("Vision");

    cams = new WallEyeCam[VisionConstants.kNumCams];
    udpSubscriber = new UdpSubscriber[VisionConstants.kNumPis];
    // We copy the matrix from our constants
    adaptiveMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();
    // We then copy the adaptive matrix because they will differ later
    stdMatrix = adaptiveMatrix.copy();
    // I'm not sure why we put this in a try catch maybe could be removed
    field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    // Fill our camera array
    for (int i = 0; i < VisionConstants.kNumCams; i++) {
      cams[i] = new WallEyeCam(camNames[i], -1);
      acceptUpdates[i] = true;
    }
    // Initialize our udpSubscribers
    udpSubscriber[0] = new UdpSubscriber(5802, cams[0]);
    udpSubscriber[1] = new UdpSubscriber(5803, cams[2]);
    udpSubscriber[2] = new UdpSubscriber(5804, cams[1], cams[3], cams[4]);
  }
  // Setter Methods
  public void setVisionUpdating(boolean updating) {
    this.visionUpdating = updating;
  }

  public void setYawUpdateCamera(int idx) {
    trustedCameraYawIdx = idx;
    Logger.recordOutput("Vision/Trusted Yaw Camera Index", trustedCameraYawIdx);
  }

  public void setMinTags(int minTags) {
    this.minTags = minTags;
  }

  public void setIgnoreRearCams(boolean ignore) {
    this.ignoreRearCams = ignore;
    acceptUpdates[1] = acceptUpdates[3] = !ignore;
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
    acceptUpdates[1] = acceptUpdates[3] = !isAuto;
  }

  public void setUsingLeftCam(boolean useLeft) {
    acceptUpdates[0] = useLeft;
  }

  public void setUsingRightCam(boolean useRight) {
    acceptUpdates[2] = useRight;
  }

  // Getter Methods
  public double getYawUpdateCamera() {
    return trustedCameraYawIdx;
  }

  public boolean isVisionUpdating() {
    return visionUpdating;
  }

  public boolean cameraConnected(int index) {
    return cams[index].isCameraConnected();
  }

  private double getSeconds() {
    return RobotController.getFPGATime() / 1_000_000;
  }

  private double minTagDistance(WallEyePoseResult result) {

    Pose3d camLoc = result.getCameraPose();
    int[] ids = result.getTagIDs();
    // This number's value doesn't really matter it just needs to be large
    double minDistance = 2000;

    // Go through the camera locations and finds the one closest to the tag
    for (int id : ids) {
      Pose3d tagPose = field.getTagPose(id).get();

      double dist =
          Math.sqrt(
              FastMath.pow2(tagPose.getX() - camLoc.getX())
                  + FastMath.pow2(tagPose.getY() - camLoc.getY()));

      if (dist < minDistance) {
        minDistance = dist;
      }
    }
    return minDistance;
  }

  private double averageTagDistance(WallEyePoseResult result) {

    Pose3d camLoc = result.getCameraPose();

    int[] ids = result.getTagIDs();
    double totalDistance = 0.0;

    // Goes through the camera locations and gets the average distance
    for (int id : ids) {
      Pose3d tagPose = field.getTagPose(id).get();
      totalDistance +=
          Math.sqrt(
              FastMath.pow2(tagPose.getX() - camLoc.getX())
                  + FastMath.pow2(tagPose.getY() - camLoc.getY()));
    }
    return totalDistance / ids.length;
  }

  // Filters
  /* Commented until a Drive Subsystem
  private boolean camsAgreeWithWheels(Translation3d pose, WallEyeTagResult result) {

    ChassisSpeeds vel = driveSubsystem.getFieldRelSpeed();
    Pose2d curPose = driveSubsystem.getPoseMeters();

    Translation2d disp = (curPose.getTranslation().minus(pose.toTranslation2d()));

    double velMagnitude =
        Math.sqrt(FastMath.pow2(vel.vxMetersPerSecond) + FastMath.pow2(vel.vyMetersPerSecond));

    double dispMagnitude = Math.sqrt(FastMath.pow2(disp.getX()) + FastMath.pow2(disp.getY()));

    /*This gets our displacement and compares it to who much we could
    have moved.It does this by getting the velocity and plotting it on a
    graph. The graph will be in the docs.
    return result.getNumTags() >= minTags
        && dispMagnitude
            <= (velMagnitude * VisionConstants.kLinearCoeffOnVelFilter
                + VisionConstants.kOffsetOnVelFilter
                + FastMath.pow2(velMagnitude * VisionConstants.kSquaredCoeffOnVelFilter));
  }*/

  private boolean camsWithinField(Translation3d pose, WallEyePoseResult result) {
    return (result.getNumTags() >= 2 || result.getAmbiguity() < VisionConstants.kMaxAmbig)
        && pose.getX() < field.getFieldLength()
        && pose.getX() > 0
        && pose.getY() < field.getFieldWidth()
        && pose.getY() > 0;
  }

  /*Large switch case to see get the standard deviation factor based on camera, how many
  tags we see and distance. An example of this graph will be in the docs.*/
  private double getStdDevFactor(double distance, int numTags, String camName) {
    switch (camName) {
      case VisionConstants.kCam2Name, VisionConstants.kCam4Name -> {
        if (numTags == 1)
          return 1
              / (VisionConstants.FOV58YUYVBaseTrust
                  * FastMath.exp(
                      FastMath.pow(
                          VisionConstants.FOV58YUYVSingleTagCoeff * distance,
                          VisionConstants.FOV58YUYVPowerNumber)));

        return 1
            / (VisionConstants.FOV58YUYVBaseTrust
                * FastMath.exp(
                    FastMath.pow(
                        VisionConstants.FOV58YUYVMultiTagCoeff * distance,
                        VisionConstants.FOV58YUYVPowerNumber)));
      }
      case VisionConstants.kCam5Name -> {
        if (numTags == 1)
          return 1
              / (VisionConstants.FOV58YUYVBaseTrust
                  * FastMath.exp(
                      FastMath.pow(
                          VisionConstants.FOV58MJPGSingleTagCoeff * distance,
                          VisionConstants.FOV58YUYVPowerNumber)));

        return 1
            / (VisionConstants.FOV58YUYVBaseTrust
                * FastMath.exp(
                    FastMath.pow(
                        VisionConstants.FOV58MJPGSingleTagCoeff * distance,
                        VisionConstants.FOV58YUYVPowerNumber)));
      }

      case VisionConstants.kCam1Name, VisionConstants.kCam3Name -> {
        if (numTags == 1)
          return 1
              / (VisionConstants.FOV75YUYVBaseTrust
                  * FastMath.exp(
                      FastMath.pow(
                          VisionConstants.FOV75YUYVSingleTagCoeff * distance,
                          VisionConstants.FOV75YUYVPowerNumber)));
        return 1
            / (VisionConstants.FOV75YUYVBaseTrust
                * FastMath.exp(
                    FastMath.pow(
                        VisionConstants.FOV75YUYVMultiTagCoeff * distance,
                        VisionConstants.FOV75YUYVPowerNumber)));
      }

      default -> {
        if (numTags == 1)
          return 1
              / (VisionConstants.baseTrust
                  * FastMath.exp(
                      FastMath.pow(
                          VisionConstants.singleTagCoeff * distance, VisionConstants.powerNumber)));
        return 1
            / (VisionConstants.baseTrust
                * FastMath.exp(
                    FastMath.pow(
                        VisionConstants.multiTagCoeff * distance, VisionConstants.powerNumber)));
      }
    }
  }

  private Pose3d getCloserPose(Pose3d pose1, Pose3d pose2, double rotation, int idx) {
    /*Which pose rotation is closer to our gyro. We subtract the absolute value of the gyro
    from the rotation of the pose and compare the two*/

    double pose1Error = 2767;
    double pose2Error = 2767;
    if (pose1 != null) {
      pose1Error =
          FastMath.abs(
              Rotation2d.fromRadians(rotation)
                  .minus(pose1.getRotation().rotateBy(camRotations[idx]).toRotation2d())
                  .getRadians());
    }
    if (pose2 != null) {
      pose2Error =
          FastMath.abs(
              Rotation2d.fromRadians(rotation)
                  .minus(pose2.getRotation().rotateBy(camRotations[idx]).toRotation2d())
                  .getRadians());
    }
    Logger.recordOutput("Vision/Pose 1 Yaw Error", pose1Error);
    Logger.recordOutput("Vision/Pose 2 Yaw Error", pose2Error);
    Logger.recordOutput("Vision/Historical yaw", rotation);

    if (pose1Error > VisionConstants.kYawErrorThreshold) {
      pose1 = null;
      // textLogger.info("Reject 1 due to yaw");
    }
    if (pose2Error > VisionConstants.kYawErrorThreshold) {
      pose2 = null;
      // textLogger.info("Reject 2 due to yaw");
    }

    if (pose1 == null && pose2 == null) {
      return null;
    }

    if (pose1Error < pose2Error) {
      // textLogger.info("Accept 1");
      return pose1;
    } else {
      // textLogger.info("Accept 2");
      return pose2;
    }
  }

  private Pose3d getCorrectPose(Pose3d pose1, Pose3d pose2, double time, int camIndex) {
    double dist1 = Math.abs(camHeights[camIndex] - pose1.getZ());
    double dist2 = Math.abs(camHeights[camIndex] - pose2.getZ());

    // // This filters out results by seeing if they are close to the right height
    if (dist1 > VisionConstants.kCamErrorZThreshold) {
      // textLogger.info("Reject 1 due to z");
      pose1 = null;
    }
    if (dist2 > VisionConstants.kCamErrorZThreshold) {
      pose2 = null;
      // textLogger.info("Reject 2 due to z");
    }

    // If we don't have enough data in the gyro buffer we default to returning a pose
    if (gyroBuffer.size() < VisionConstants.kCircularBufferSize) {
      if (pose1 != null) {
        return pose1;
      } else {
        return pose2;
      }
    }

    // See what pose is closer the the gyro at the time of the photo's capture.
    double rotation =
        gyroBuffer.get(
            FastMath.floorToInt(
                (((RobotController.getFPGATime() - time) / 1_000_000.0)
                    / VisionConstants.kLoopTime)));
    Logger.recordOutput(
        "Vision/Gyro Queried Loop Count",
        FastMath.floorToInt(((time / 1_000_000.0) / VisionConstants.kLoopTime)));
    return getCloserPose(pose1, pose2, rotation, camIndex);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Vision/Vision Updates On", visionUpdating);
    double gyroData = 0.0;
    // FastMath.normalizeMinusPiPi(turretSubsystem.getGyroRotation2d().getRadians());
    gyroBuffer.addFirst(gyroData);

    Logger.recordOutput("Vision/Gyro Buffer", gyroData);

    if (getSeconds() - timeSinceLastUpdate > VisionConstants.kMaxTimeNoVision) {
      updatesToWheels = 0;
    }

    validResults.clear();

    for (int i = 0; i < VisionConstants.kNumCams; i++) {
      if (!ignoreRearCams && !isAuto || (i == 0 || i == 2) && acceptUpdates[i]) {
        if (cams[i].hasNewUpdate()) {
          timeSinceLastUpdate = getSeconds();
          validResults.add(new Pair<WallEyeResult, Integer>(cams[i].getResults(), i));
          lastResult[i] = (WallEyeTagResult) cams[i].getResults();
        }
      }
    }

    if (getSeconds() - timeSinceLastUpdate >= VisionConstants.kTimeToDecayDev) {
      // Decrease the thresholds required for a good pose over time. A graph is in the docs
      for (int i = 0; i < 2; i++) {
        double scaledWeight =
            VisionConstants.kVisionMeasurementStdDevs.get(i, 0)
                + VisionConstants.kStdDevDecayCoeff
                    * ((getSeconds() - timeSinceLastUpdate) - VisionConstants.kTimeToDecayDev);

        adaptiveMatrix.set(
            i,
            0,
            scaledWeight >= VisionConstants.kMinStdDev ? scaledWeight : VisionConstants.kMinStdDev);
      }
    }

    for (Pair<WallEyeResult, Integer> res : validResults) {
      if (res.getFirst() instanceof WallEyePoseResult) {
        /*Reset the adaptive matrix to a stricter value once we get a result.
        We set it to a stricter value then initially.*/
        adaptiveMatrix.set(0, 0, .1);
        adaptiveMatrix.set(1, 0, .1);

        WallEyePoseResult result = (WallEyePoseResult) res.getFirst();

        if (result.getCameraPose() == null) { // TODO figure out why it sometimes is null
          textLogger.error("Pose is null, skipping!");
          continue;
        }

        int idx = res.getSecond();

        for (int i = 0; i < 2; i++) {
          stdMatrix.set(
              i,
              0,
              .1 / getStdDevFactor(minTagDistance(result), result.getNumTags(), camNames[idx]));
        }

        Pose3d cameraPose;
        Translation3d robotTranslation;
        Rotation3d cameraRotation;

        if (result.getNumTags() > 1) {
          // If there our more then one tag in an image we can get pose possible pose
          cameraPose = result.getCameraPose();

          cameraRotation = cameraPose.getRotation().rotateBy(camRotations[idx]);
          robotTranslation =
              cameraPose.getTranslation().minus(camPositions[idx].rotateBy(cameraRotation));

        } else {
          // If there is one we get two possible poses and have to filter them.
          Pose3d cam1Pose = result.getFirstPose();
          Pose3d cam2Pose = result.getSecondPose();

          Logger.recordOutput("Vision/Raw Camera 1 " + camNames[idx], cam1Pose);
          Logger.recordOutput("Vision/Raw Camera 2 " + camNames[idx], cam2Pose);

          // textLogger.info("Processing camera {}", camNames[idx]);
          cameraPose = getCorrectPose(cam1Pose, cam2Pose, result.getTimeStamp(), idx);

          if (cameraPose == null) {
            // textLogger.info("Both poses rejected!");
            continue;
          }

          cameraRotation = cameraPose.getRotation().rotateBy(camRotations[idx]);
          robotTranslation =
              cameraPose.getTranslation().minus(camPositions[idx].rotateBy(cameraRotation));
        }
        Pose2d robotPose =
            new Pose2d(robotTranslation.toTranslation2d(), cameraRotation.toRotation2d());
        if (camsWithinField(robotTranslation, result)) {
          // Is the pose in the field? If so, enjoy a updated position drive subsystem
          updatesToWheels++;
          Logger.recordOutput("Vision/Accepted Cam " + camNames[idx], robotPose);
          // However we do have to be accepting the poses to use them
          if (visionUpdating) {
            if (idx == trustedCameraYawIdx) {
              stdMatrix.set(2, 0, VisionConstants.kTrustYawStdDev);
            }

            // robotState.addVisionMeasurement(
            //     robotPose, result.getTimeStamp() / 1_000_000.0, stdMatrix);
            stdMatrix.set(2, 0, VisionConstants.kIgnoreYawStdDev);
          }
        } else {
          Logger.recordOutput("Vision/Rejected Cam " + camNames[idx], robotPose);
        }
      }
    }
  }

  public WallEyeTagResult getLastResult(int index) {
    return lastResult[index];
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }
}
