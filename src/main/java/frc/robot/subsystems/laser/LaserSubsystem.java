package frc.robot.subsystems.laser;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.LaserConstants;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LaserSubsystem extends MeasurableSubsystem {
  private LaserSubsystemIO io;
  private LaserIOInputsAutoLogged inputs = new LaserIOInputsAutoLogged();
  private Angle setPoint;

  public LaserSubsystem() {
    this.io = io;
  }

  public Angle getLaserPos() {
    return Rotations.of(inputs.position);
  }

  public void setPosition(Angle pos) {
    setPoint = pos;
    io.setPosition(pos);
  }

  public void zero() {
    // TODO Add kraken encoder and make zero function
  }

  public boolean isFinished() {
    return Math.abs(getLaserPos().minus(setPoint).in(Rotations)) < LaserConstants.kLaserCloseEnough;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Turret Finished?", () -> isFinished() ? 1 : 0),
        new Measure("Turret Setpoint", () -> setPoint.in(Rotations)));
  }
}
