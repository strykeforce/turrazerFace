package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystemIO.TurretIOInputs;
import org.strykeforce.telemetry.TelemetryService;

public class TurretSubsystemIOFX implements TurretSubsystemIO {

  private TalonFX talonFX;
  private TalonFXConfigurator talonConfigurator;
  private CANcoder canCoder1; // left
  private CANcoder canCoder2; // right

  StatusSignal<Angle> curPosition;
  StatusSignal<AngularVelocity> curVelocity;

  private MotionMagicVoltage positionMain =
      // Update MotionMagic constants
      new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);

  public TurretSubsystemIOFX() {
    talonFX = new TalonFX(TurretConstants.kTurretFx);
    talonConfigurator = talonFX.getConfigurator();
    talonConfigurator.apply(TurretConstants.turretFXConfig());
    canCoder1 = new CANcoder(TurretConstants.kCanCoder1Id);
    canCoder2 = new CANcoder(TurretConstants.kCanCoder2Id);

    curPosition = talonFX.getPosition();
    curVelocity = talonFX.getVelocity();
  }

  public void zeroTurret() {}

  public void setPosition(Angle position) {
    talonFX.setControl(positionMain.withPosition(position));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        curPosition, curVelocity, canCoder1.getPosition(), canCoder2.getPosition());
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFX, true);
    telemetryService.register(canCoder1);
    telemetryService.register(canCoder2);
  }
}
