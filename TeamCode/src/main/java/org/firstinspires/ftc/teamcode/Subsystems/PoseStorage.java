package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    // This variable stays alive even when OpModes switch
    public static Pose2d currentPose = new Pose2d(0, 0, 0);
}