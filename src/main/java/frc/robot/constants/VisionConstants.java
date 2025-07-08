package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import net.jafama.FastMath;

public final class VisionConstants {
  public static final double kMaxTimeNoVision = 20;
  public static final double kTimeToResetWheelCount = 5;
  public static final int kResultsForWheels = 5;
  public static final double kTimeToDecayDev = 10;
  public static final double kStdDevDecayCoeff = -0.005;
  public static final double kMinStdDev = 0.01;
  public static final double kMaxAmbig = 1.0;
  public static final int kMaxTimesOffWheels = 5;
  public static final double kBumperPixelLine = 87; // 100
  public static final double kRobotHeight = 0.5;

  //   public static final double kThetaStdDevUsed = Units.degreesToRadians(0.02);
  //   public static final double kThetaStdDevRejected = Units.degreesToRadians(360);
  //   public static final double kThetaStdThres = 0.2;

  // Velocity Filter
  public static final double kLinearCoeffOnVelFilter = 0.1;
  public static final double kOffsetOnVelFilter = 0.10;
  public static final double kSquaredCoeffOnVelFilter = 0.1;

  public static Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.0);

  public static final double kTimeStampOffset = 0.0;

  // StdDev scaling
  public static final double singleTagCoeff = 25.0 / 100.0;
  public static final double multiTagCoeff = 18.0 / 100.0;
  public static final double baseNumber = Math.E;
  public static final double powerNumber = 2.0;
  public static final double baseTrust = 3.0;

  public static final double FOV45MultiTagCoeff = 16.0 / 100.0;
  public static final double FOV45powerNumber = 4.5;
  public static final double FOV45SingleTagCoeff = 21.0 / 100.0;
  public static final double FOV45BaseTrust = 2.0;

  public static final double FOV58MJPGMultiTagCoeff = 16.0 / 100.0;
  public static final double FOV58MJPGPowerNumber = 3.5;
  public static final double FOV58MJPGSingleTagCoeff = 21.0 / 100.0;
  public static final double FOV58MJPGBaseTrust = 3.0;

  public static final double FOV58YUYVMultiTagCoeff = 17.0 / 100.0;
  public static final double FOV58YUYVPowerNumber = 2.0;
  public static final double FOV58YUYVSingleTagCoeff = 22.0 / 100.0;
  public static final double FOV58YUYVBaseTrust = 3.0;

  public static final double FOV75YUYVMultiTagCoeff = 17.0 / 100.0;
  public static final double FOV75YUYVPowerNumber = 2.0;
  public static final double FOV75YUYVSingleTagCoeff = 22.0 / 100.0;
  public static final double FOV75YUYVBaseTrust = 3.0;

  // Gyro error
  public static final double kYawErrorThreshold = Units.degreesToRadians(30);
  public static final double kCamErrorZThreshold = 0.3;

  // Constants for cameras
  public static final int kNumCams = 5;
  public static final int kNumPis = 3;
  public static final int[] kUdpIndex = {0, 1, 2};

  // Camera Ports
  public static final int[] kCamPorts = {5802, 5804, 5803, 5804, 5804};

  // Names
  public static final String kCam1Name = "Left Servo";
  public static final String kCam2Name = "Rear Right"; // when looking out the back
  public static final String kCam3Name = "Right Servo";
  public static final String kCam4Name = "Rear Left";
  public static final String kCam5Name = "Rear";

  //   public static final String kPi1Name = "Left";
  //   public static final String kPi2Name = "Right";
  //   public static final String kPi3Name = "Rear";

  // Indexs
  //   public static final int kCam1Idx = 0;
  //   public static final int kCam2Idx = 0;
  //   public static final int kCam3Idx = 0;
  //   public static final int kCam4Idx = 0;
  //   public static final int kCam5Idx = 2;

  public static final double kLoopTime = 0.02;
  public static final int kCircularBufferSize = 1000;
  // Poses
  public static final Pose3d kCam1Pose =
      new Pose3d(new Translation3d(0.305, 0.0255, 0.311), new Rotation3d());
  public static final Pose3d kCam2Pose =
      new Pose3d(
          new Translation3d(-0.275, -0.21, 0.49),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(145)));
  public static final Pose3d kCam3Pose =
      new Pose3d(
          new Translation3d(0.133, -0.3, 0.311), // -0.305
          new Rotation3d(0, Units.degreesToRadians(-2), Units.degreesToRadians(1)));
  public static final Pose3d kCam4Pose =
      new Pose3d(
          new Translation3d(-0.275, -0.145, 0.49),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-145)));
  public static final Pose3d kCam5Pose =
      new Pose3d(
          new Translation3d(-0.229, 0.073, 0.934),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(180)));
  // Increase these numbers to trust sensor readings from encoders and gyros less. This matrix is
  // in the form [theta], with units in radians.
  public static final Matrix<N1, N1> kLocalMeasurementStdDevs =
      VecBuilder.fill(Units.degreesToRadians(0.01));

  public static final double kIgnoreYawStdDev = 10000 * FastMath.PI;
  public static final double kTrustYawStdDev = 0.0005;
  // Increase these numbers to trust global measurements from vision less. This matrix is in the
  // form [x, y, theta]ᵀ, with units in meters and radians.
  // Vision Odometry Standard devs
  public static final Matrix<N3, N1> kVisionMeasurementStdDevs =
      VecBuilder.fill(0.05, 0.05, kIgnoreYawStdDev);
}
