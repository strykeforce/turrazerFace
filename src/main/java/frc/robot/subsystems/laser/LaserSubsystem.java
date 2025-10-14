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
  private LaserStates curState;
  private int zeroCounter = 0;

  public LaserSubsystem(LaserSubsystemIO io) {
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
    io.setOpenLoopVoltage(LaserConstants.kZeroingVolts);
    io.setSoftLimitConfig(LaserConstants.getZeroingSoftLimitConfigs());
    curState = LaserStates.ZEROING;
  }
  // TODO Find whats up with these functions not needing @Overide
  public boolean isFinished() {
    return Math.abs(getLaserPos().minus(setPoint).in(Rotations)) < LaserConstants.kLaserCloseEnough
        && curState != LaserStates.ZEROING;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Turret Finished?", () -> isFinished() ? 1 : 0),
        new Measure("Turret Setpoint", () -> setPoint.in(Rotations)));
  }

  public void perodic() {
    io.updateInputs(inputs);

    switch (curState) {
      case ZEROING:
        if (Math.abs(inputs.velocity) < LaserConstants.kLaserCloseEnough) {
          zeroCounter++;
          if (zeroCounter <= LaserConstants.kZeroCounter) {
            io.zero();
            zeroCounter = 0;
            io.setLimitConfig(LaserConstants.laserFXConfig().CurrentLimits);
            setPosition(Rotations.of(0));
          }
        }
        break;

      case ZEROED:
        break;
    }
  }

  public enum LaserStates {
    ZEROING,
    ZEROED
  }
}
