package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface TurretSubsystemIO {

  @AutoLog
  public class TurretIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
