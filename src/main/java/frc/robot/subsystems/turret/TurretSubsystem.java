package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.TurretConstants;
import java.util.ArrayList;
import java.util.Set;
import net.jafama.FastMath;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class TurretSubsystem extends MeasurableSubsystem {

  private TurretSubsystemIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double setPoint;
  private ArrayList<Double> possiblePos1 = new ArrayList<Double>();
  private ArrayList<Double> possiblePos2 = new ArrayList<Double>();

  public TurretSubsystem(TurretSubsystemIO io) {
    this.io = io;
  }

  public void setPosition(double setPos) {
    setPoint = setPos;
    io.setPosition(checkForWrap(setPoint));
  }

  public void pointAtPos(Pose2d tarPos, Pose2d drivePos) {
    /*1 Get x,y delta from our target to our robot with .minus
    returning a translation2d.
    2 Get the angle from the translation2d.
    3. Subtract our pose meters from drive.
    */
    double targetAngle =
        tarPos
            .minus(drivePos)
            .getTranslation()
            .getAngle()
            .minus(drivePos.getRotation())
            .getMeasure()
            .in(Rotations);
    setPosition(targetAngle);
  }

  public double getPosition() {
    return inputs.position;
  }
  // Work on robot state. Do the cool stuff. I believe in you .
  public double checkForWrap(double tarPos) {
    // Convert back to angle
    tarPos = tarPos * TurretConstants.kMotorGr;
    if (tarPos < TurretConstants.kWrap1) {
      return (360.0 - tarPos) / TurretConstants.kMotorGr;
    } else if (tarPos > TurretConstants.kWrap2) {
      return (tarPos - 360) / TurretConstants.kMotorGr;
    } else return tarPos / TurretConstants.kMotorGr;
    // Convert back to rotations because I'm tired and didn't want to do anything differently.
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
    return Math.abs(getPosition() - setPoint) < TurretConstants.kTurretCloseEnough;
  }
@Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Turret Finished?", () -> isFinished() ? 1 : 0),
        new Measure("Turret Setpoint", () -> setPoint),
        new Measure("Turret Zero Point", () -> zero()));
  }
}
