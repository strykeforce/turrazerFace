package frc.robot.subsystems.laser;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.LaserConstants;
import org.strykeforce.telemetry.TelemetryService;

public class LaserSubsystemIOFX implements LaserSubsystemIO {

  private MotionMagicVoltage motionMagic =
      new MotionMagicVoltage(0).withEnableFOC(false).withSlot(1);
  private TalonFX talonFX;
  private TalonFXConfigurator talonFXConfigurator;
  private StatusSignal<Angle> position;

  public LaserSubsystemIOFX() {
    talonFX = new TalonFX(LaserConstants.kLaserFxId);
    talonFXConfigurator = talonFX.getConfigurator();
    talonFXConfigurator.apply(LaserConstants.turretFXConfig());

    position = talonFX.getPosition();
  }

  public void setPosition(Angle setPos) {
    talonFX.setControl(motionMagic.withPosition(setPos));
  }

  @Override
  public void updateInputs(LaserIOInputs input) {
    BaseStatusSignal.refreshAll(position);
  }

  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFX, true);
  }
}
