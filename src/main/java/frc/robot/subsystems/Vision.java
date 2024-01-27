package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.util.Limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PoseEstimator s_PoseEstimator;
    public Limelight leftLL;
    public Limelight rightLL;

    public Vision(PoseEstimator s_PoseEstimator) {
        this.s_PoseEstimator = s_PoseEstimator;
        leftLL = new Limelight(Constants.Vision.leftLLName);
        rightLL = new Limelight(Constants.Vision.rightLLName);
    }

    public boolean rightHasTarget(){
        return (rightLL.getBlueBotPose2d().getX() != 0 && rightLL.getBlueBotPose2d().getY() != 0);
    }

    public boolean leftHasTarget(){
        return (leftLL.getBlueBotPose2d().getX() != 0 && leftLL.getBlueBotPose2d().getY() != 0);
    }
    
    public void updateVision(){
        double Lx = leftLL.getRedBotPose2d().getX();
        double Ly = leftLL.getRedBotPose2d().getY();
        double Rx = rightLL.getBlueBotPose2d().getX();
        double Ry = rightLL.getBlueBotPose2d().getY();
        boolean Ltarget = (Lx != 0 && Ly != 0);
        boolean Rtarget = (Rx != 0 && Ry != 0);
        if(Ltarget && Rtarget){
            Pose2d LLpose = new Pose2d((Lx + Rx) / 2.0, (Ly + Ry) / 2.0, Rotation2d.fromDegrees(0.0));
            double LLlatency = (leftLL.getLatency() + rightLL.getLatency()) / 2;
            s_PoseEstimator.updateVision(LLpose, LLlatency);         
        }else if(Ltarget){
            Pose2d LLpose = new Pose2d(Lx, Ly, Rotation2d.fromDegrees(0.0));
            double LLlatency = leftLL.getLatency();
            s_PoseEstimator.updateVision(LLpose, LLlatency);  
        }else if(Rtarget){
            Pose2d LLpose = new Pose2d(Rx, Ry, Rotation2d.fromDegrees(0.0));
            double LLlatency = rightLL.getLatency();
            s_PoseEstimator.updateVision(LLpose, LLlatency);         
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("left LL, X", leftLL.getBlueBotPose2d().getX());
        SmartDashboard.putNumber("left LL, Y", leftLL.getBlueBotPose2d().getY());
        SmartDashboard.putNumber("right LL, X", rightLL.getBlueBotPose2d().getX());
        SmartDashboard.putNumber("right LL, Y", rightLL.getBlueBotPose2d().getY());
        Logger.recordOutput("LLLeft Pose", leftLL.getBlueBotPose2d());
        
         if(s_PoseEstimator.readyToUpdateVision()){
            updateVision();
        } 
        
    }
}