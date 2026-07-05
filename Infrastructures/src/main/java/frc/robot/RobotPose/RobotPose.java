package frc.robot.RobotPose;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.robot.RobotPose.Vision.VisionSource;

public class RobotPose {
    private static RobotPose instance;

    private final DemaciaPoseEstimator poseEstimator;
    private final ArrayList<VisionSource> visionSources;
    private Supplier<OdometryData> odometryDataSupplier;

    private RobotPose(Supplier<OdometryData> supplier, SwerveModulePosition[] initialModulePositions){
        this.odometryDataSupplier = supplier;
        this.poseEstimator = new DemaciaPoseEstimator(initialModulePositions, LocalizationConfig., null)

    }
    
}
