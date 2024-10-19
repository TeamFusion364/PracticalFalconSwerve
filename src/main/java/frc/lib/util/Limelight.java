package frc.lib.util;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Limelight {
    private final String LLNTname;

    /**
     * @param LLNTname The name of the Limelight Network Table
     */
    public Limelight(String LLNTname) {
        this.LLNTname = "limelight-" + LLNTname;
    }

    /**@return Horizontal Offset From Crosshair To Target in degrees
     * (Note: Inverted from standard LL provided angle to be CCW+) */
    public Rotation2d getRetroTx() {
        return Rotation2d.fromDegrees(-NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("tx").getDouble(0));
    }

    /**@return Vertical Offset from Crosshair to Target in degrees */
    public Rotation2d getRetroTy() {
        return Rotation2d.fromDegrees(NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("ty").getDouble(0));
    }

    /**
     * Checks for Target, and also resets distanceFilter if no target
     * @return True if LL detects Target 
     */
    public boolean hasRetroTarget(){
        boolean hasTarget = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("tv").getDouble(0) == 1;
        return hasTarget;
    }

    /**@return The pipeline’s latency contribution in seconds */
    public double getLatency() {
        double pipelineLatency = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("tl").getDouble(0) / 1000;
        double captureLatency = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("cl").getDouble(0) / 1000;
        return (pipelineLatency + captureLatency);
    }

    /**@return Get BotPose from April Tags */
    public Pose2d getBotPose2d() {
        double[] botPose = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("botpose").getDoubleArray(new double[6]);
        Pose2d botPose2d = new Pose2d(new Translation2d(botPose[0], botPose[1]), Rotation2d.fromDegrees(botPose[5]));
        return botPose2d;
    }

    /**@return Get BotPose from April Tags */
    public Pose3d getBotPose3d() {
        double[] botPose = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("botpose").getDoubleArray(new double[6]);
        Pose3d botPose3d = new Pose3d(
            new Translation3d(botPose[0], botPose[1], botPose[2]), 
            new Rotation3d(
                Units.degreesToRadians(botPose[3]), 
                Units.degreesToRadians(botPose[4]), 
                Units.degreesToRadians(botPose[5])
        ));
        return botPose3d;
    }

    /**@return Get BotPose from April Tags */
    public Pose2d getBlueBotPose2d() {
        double[] botPose = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        Pose2d botPose2d = new Pose2d(new Translation2d(botPose[0], botPose[1]), Rotation2d.fromDegrees(botPose[5]));
        return botPose2d;
    }

    /**@return Get BotPose from April Tags */
    public Pose3d getBlueBotPose3d() {
        double[] botPose = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        Pose3d botPose3d = new Pose3d(
            new Translation3d(botPose[0], botPose[1], botPose[2]), 
            new Rotation3d(
                Units.degreesToRadians(botPose[3]), 
                Units.degreesToRadians(botPose[4]), 
                Units.degreesToRadians(botPose[5])
        ));
        return botPose3d;
    }

    /**@return Get BotPose from April Tags */
    public Pose2d getRedBotPose2d() {
        double[] botPose = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("botpose_wpired").getDoubleArray(new double[6]);
        Pose2d botPose2d = new Pose2d(new Translation2d(botPose[0], botPose[1]), Rotation2d.fromDegrees(botPose[5]));
        return botPose2d;
    }

    /**@return Get BotPose from April Tags */
    public Pose3d getRedBotPose3d() {
        double[] botPose = NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("botpose_wpired").getDoubleArray(new double[6]);
        Pose3d botPose3d = new Pose3d(
            new Translation3d(botPose[0], botPose[1], botPose[2]), 
            new Rotation3d(
                Units.degreesToRadians(botPose[3]), 
                Units.degreesToRadians(botPose[4]), 
                Units.degreesToRadians(botPose[5])
        ));
        return botPose3d;
    }

    /**
     * Use to set state of LL's leds
     */
    public void ledState(ledStates ledState){
        NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("ledMode").setNumber(ledState.value);
    }

    /** Use to set LL camera mode
     * @param camMode
     * <pre>
     * 0: driver
     * 1: vision
     */
    public void camMode(camModes camMode){
        NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("camMode").setNumber(camMode.value);
    }

    /** Use to set active pipeline
     * @param pipeline 0-9
     */
    public void setPipeline(int pipeline){
        NetworkTableInstance.getDefault().getTable(LLNTname).getEntry("pipeline").setNumber(pipeline);
    }
    
    /** Possible values for Led States. */
    public enum ledStates {
        pipeline(0),
        off(1),
        blink(2),
        on(3);
        
        public final int value;
        ledStates(int value) {
          this.value = value;
        }
    }

    
    /** Possible values for Cam Modes. */
    public enum camModes {
        vision(0),
        driver(1);
        
        public final int value;
        camModes(int value) {
          this.value = value;
        }
    }
}
