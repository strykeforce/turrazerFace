package frc.robot.subsystems.laser;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface LaserSubsystemIO {

  @AutoLog
  public class LaserIOInputs {
    public double position;
  }

  public default void updateInputs(LaserIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setPosition(Angle setPos) {}
}
