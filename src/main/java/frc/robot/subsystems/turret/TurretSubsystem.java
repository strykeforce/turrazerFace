package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.TurretConstants;
import java.util.ArrayList;
import java.util.Set;
import net.jafama.FastMath;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class TurretSubsystem extends MeasurableSubsystem {

  private TurretSubsystemIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Angle setPoint;
  private ArrayList<Double> possiblePos1 = new ArrayList<Double>();
  private ArrayList<Double> possiblePos2 = new ArrayList<Double>();

  public TurretSubsystem() {
    this.io = io;
  }

  public void setPosition(Angle setPos) {
    setPoint = setPos;
    io.setPosition(setPoint);
  }

  public Angle getPosition() {
    return Rotations.of(inputs.position);
  }

  private boolean withInTolerence(double pos1, double pos2) {
    return Math.abs(pos1 - pos2) < TurretConstants.kZeroDifferenceTol;
  }

  public double zero() {

    for (int i = 0; i < 8; i++) {
      possiblePos1.add(
          (FastMath.abs(inputs.canCoder1Pos.in(Rotations) - TurretConstants.kCanCoder1Zero))
                  * TurretConstants.kCanCoder1Gr
              + TurretConstants.kCanCoder1PosRot[i]);
    }
    for (int i = 0; i < 8; i++) {
      possiblePos1.add(
          FastMath.abs(inputs.canCoder2Pos.in(Rotations) - TurretConstants.kCanCoder2Zero)
                  * TurretConstants.kCanCoder2Gr
              + TurretConstants.kCanCoder2PosRot[i]);
    }

    for (int i = 0; i < 8; i++) {
      for (int j = 0; j < 8; j++) {
        if (withInTolerence(possiblePos1.get(i), possiblePos2.get(j))) {
          return possiblePos1.get(i);
        }
      }
    }
    return 2767;
  }

  public boolean isFinished() {
    return Math.abs(getPosition().minus(setPoint).in(Rotations))
        < TurretConstants.kTurretCloseEnough;
  }

  public void perodic() {
    io.updateInputs(inputs);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Turret Finished?", () -> isFinished() ? 1 : 0),
        new Measure("Turret Setpoint", () -> setPoint.in(Rotations)),
        new Measure("Turret Zero Point", () -> zero()));
  }
}
