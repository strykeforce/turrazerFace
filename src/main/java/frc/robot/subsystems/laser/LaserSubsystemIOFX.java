package frc.robot.subsystems.laser;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.LaserConstants;
import org.strykeforce.telemetry.TelemetryService;

public class LaserSubsystemIOFX implements LaserSubsystemIO {

  private MotionMagicVoltage motionMagic =
      new MotionMagicVoltage(0).withEnableFOC(false).withSlot(1);
  private TalonFX talonFX;
  private TalonFXConfigurator talonFXConfigurator;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private DutyCycleOut openLoopVelocity = new DutyCycleOut(0);
  private VoltageOut openLoopVoltage = new VoltageOut(0);

  public LaserSubsystemIOFX() {
    talonFX = new TalonFX(LaserConstants.kLaserFxId);
    talonFXConfigurator = talonFX.getConfigurator();
    talonFXConfigurator.apply(LaserConstants.laserFXConfig());

    position = talonFX.getPosition();
    velocity = talonFX.getVelocity();
  }

  public void setPosition(Angle setPos) {
    talonFX.setControl(motionMagic.withPosition(setPos));
  }

  public void zero() {
    setPosition(Angle.ofBaseUnits(0.0, Rotations));
    setOpenLoopVelocity(0.0);
  }

  @Override
  public void updateInputs(LaserIOInputs input) {
    BaseStatusSignal.refreshAll(position);
    BaseStatusSignal.refreshAll(velocity);
  }

  public void setLimitConfig(CurrentLimitsConfigs config) {
    talonFXConfigurator.apply(config);
  }

  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFX, true);
  }

  public void setOpenLoopVoltage(double output) {
    talonFX.setControl(openLoopVoltage.withOutput(output));
  }

  public void setOpenLoopVelocity(double output) {
    talonFX.setControl(openLoopVelocity.withOutput(output));
  }

  public void setSoftLimitConfig(SoftwareLimitSwitchConfigs config) {
    talonFXConfigurator.apply(config);
  }
}
