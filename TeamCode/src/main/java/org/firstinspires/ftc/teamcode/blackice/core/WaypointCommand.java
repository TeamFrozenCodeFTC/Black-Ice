package org.firstinspires.ftc.teamcode.blackice.core;

import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.core.commands.PathFinishCondition;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

public class WaypointCommand extends Command {
    final Pose endPose;
    final HeadingInterpolator headingInterpolator;
    final PathFinishCondition finishCondition;
    final Follower follower;
    final double followingPower;
    
    public WaypointCommand(Pose endPose, HeadingInterpolator headingInterpolator,
                           PathFinishCondition finishCondition, Follower follower, double followingPower) {
        this.endPose = endPose;
        this.headingInterpolator = headingInterpolator;
        this.finishCondition = finishCondition;
        this.follower = follower;
        this.followingPower = followingPower;
    }
    
    @Override
    public void start() {
    
    }
    
    @Override
    public void update() {
        follower.holdPose(endPose, followingPower);
    }
    
    @Override
    public boolean isFinished() {
        return finishCondition.isFinished(follower, endPose);
    }
}
