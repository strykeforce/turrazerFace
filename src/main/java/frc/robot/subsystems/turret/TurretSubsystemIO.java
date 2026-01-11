package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface TurretSubsystemIO {

  @AutoLog
  public class TurretIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public Angle canCoder1Pos;
    public Angle canCoder2Pos;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setPosition(Double setPos) {}
}
