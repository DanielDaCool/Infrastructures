// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose.Vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/** Add your docs here. */
public class Quest {
    private QuestNav questNav;
    private Pose3d currentQuestPose;
    private double timestamp;
    private Transform3d robotToQuest;
    private PoseFrame[] questFrames;

    public Quest(Transform3d robotToQuest) {
        timestamp = 0;
        questNav = new QuestNav();
        currentQuestPose = Pose3d.kZero;
        this.robotToQuest = robotToQuest;
    }

    public void setQuestPose(Pose3d robotPose) {
        questNav.setPose(robotPose.plus(robotToQuest));
    }

    public Pose2d getRobotPose() {
        return currentQuestPose.toPose2d();
    }

    public double getTimestamp(){
        return timestamp;
    }
    public boolean isConnected(){
        return questNav.isConnected();
    }
    public boolean isTracking(){
        return questNav.isTracking();
    }


    public void periodic() {

        questNav.commandPeriodic();

        questFrames = questNav.getAllUnreadPoseFrames();
        if (questFrames.length > 0) {
            currentQuestPose = questFrames[questFrames.length - 1].questPose3d().transformBy(robotToQuest.inverse());
            timestamp = questFrames[questFrames.length - 1].dataTimestamp();
        }

    }
}
