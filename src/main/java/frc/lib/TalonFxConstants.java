package frc.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Organizes the various major CTREConfigs of a TalonFX motor for easier and bloatless initialization through LazyTalonFX
 */
public class TalonFXConstants {
    public final int deviceNumber;
    public final TalonFXConfiguration allConfigs;
    public final NeutralModeValue neutralModeVal;
    public final InvertedValue invertTypeVal;
    public final boolean slowStatusFrame;    
    
    /**
     * Constants to be used with LazyTalonFX Util
     * @param deviceNumber
     * @param allConfigs
     * @param neutralModeVal
     * @param invertTypeVal
     * @param slowStatusFrame
     */
    public TalonFXConstants(int deviceNumber, TalonFXConfiguration allConfigs, NeutralModeValue neutralModeVal, InvertedValue invertTypeVal, boolean slowStatusFrame) {
        this.deviceNumber = deviceNumber;
        this.allConfigs = allConfigs;
        this.neutralModeVal = neutralModeVal;
        this.invertTypeVal = invertTypeVal;
        this.slowStatusFrame = slowStatusFrame;
    }
}
