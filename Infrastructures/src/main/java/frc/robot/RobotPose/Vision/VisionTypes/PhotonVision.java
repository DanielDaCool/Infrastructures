package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.List;

import frc.robot.RobotPose.Vision.BaseVisionSource;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.visionConfigs.PhotonVisionConfig;

public class PhotonVision extends BaseVisionSource{

    
    public PhotonVision(PhotonVisionConfig config) {
        super(config);
    }


    @Override
    public boolean shouldUpdate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'shouldUpdate'");
    }

    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPoseEstimates'");
    }

    @Override
    public boolean isConnected() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isConnected'");
    }

    
}