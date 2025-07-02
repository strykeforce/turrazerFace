package frc.robot.subsystems.vision;

import WallEye.*;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class WalleyeSubsystem extends MeasurableSubsystem{

    private WallEyeCam[] cams;
    private Translation3d[] camPositions;
    private double[] camZs;
    private Rotation3d[] camRotations;
    private String[] camNames;
    private int[] camIndexs;
    private UdpSubscriber[] udpSubscribers;

    private Logger logger;
    private LoggerFactory textLogger;
    private CircularBuffer turrentBuffer = new CircularBuffer<>(1000);
    private AprilTagFieldLayout field;

    public WalleyeSubsystem(){
       // field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        cams = new WallEyeCam[VisionConstants.kNumCams];

        for(int i = 0; i < VisionConstants.kNumCams; i++){
            cams[i] = new WallEyeCam(camNames[i], -1);
            
        }
    }


@Override
public Set<Measure> getMeasures() {
    return null;
}
}