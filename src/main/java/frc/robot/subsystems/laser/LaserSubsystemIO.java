package frc.robot.subsystems.laser;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface LaserSubsystemIO {

  @AutoLog
  public class LaserIOInputs {
    public double position;
    public double velocity;
  }

  public default void updateInputs(LaserIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setPosition(Angle setPos) {}

  public default void zero() {}

  public default void setOpenLoopVoltage(double output) {}

  public default void setOpenLoopVelocity(double output) {}

  public default void setLimitConfig(CurrentLimitsConfigs config) {}
}
