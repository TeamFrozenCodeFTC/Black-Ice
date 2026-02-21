package org.firstinspires.ftc.teamcode.blackice.core;

import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

public interface MotionController {
    void update(Follower follower);
    Pose getEndPose();
}
