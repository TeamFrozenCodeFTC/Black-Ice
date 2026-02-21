package org.firstinspires.ftc.teamcode.blackice.core;

import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

public class HoldPoseController implements MotionController {

    private final Pose target;
    private final double followingPower;

    public HoldPoseController(Pose target, double followingPower) {
        this.target = target;
        this.followingPower = followingPower;
    }

    @Override
    public void update(Follower follower) {
        follower.holdPose(target, followingPower);
    }

    @Override
    public Pose getEndPose() {
        return target;
    }
}
