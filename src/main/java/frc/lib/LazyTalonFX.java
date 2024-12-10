package frc.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

/** Thin Falcon wrapper to make setup easier. */
public class LazyTalonFX extends TalonFX {

  /**
   * Config using individual parameters. This typically should only be used for Swerve modules as
   * they have their own SwerveModuleConstants file.
   *
   * @param deviceNumber
   * @param allConfigs CTREConfig object
   * @param slowStatusFrame
   * @deprecated
   */
  public LazyTalonFX(int deviceNumber, TalonFXConfiguration allConfigs, boolean slowStatusFrame) {
    super(deviceNumber);
    TalonFXConfigurator configurator = super.getConfigurator();
    configurator.apply(allConfigs);

    if (slowStatusFrame) {
      super.getPosition().setUpdateFrequency(255, 30);
    }
  }

  /**
   * Config using individual parameters. This should only be used for Swerve modules as they have
   * their own SwerveModuleConstants file.
   *
   * @param talonFXConstants Contains various motor initialization vars
   * @param slowStatusFrame
   */
  public LazyTalonFX(TalonFxConstants talonFXConstants) {
    super(talonFXConstants.deviceNumber);
    super.getConfigurator().apply(talonFXConstants.allConfigs);

    if (talonFXConstants.slowStatusFrame) {
      super.getPosition().setUpdateFrequency(255, 30);
    }
  }
}
