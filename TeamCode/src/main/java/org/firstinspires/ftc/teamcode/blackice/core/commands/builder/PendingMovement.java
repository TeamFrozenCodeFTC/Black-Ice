package org.firstinspires.ftc.teamcode.blackice.core.commands.builder;

import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

abstract class PendingMovement implements PendingCommand {
    
    Pose endPose;
    double followingPower = 1;
    Runnable whileFollowingAction = () -> {};
    
    @Override
    public Pose getEndPose(Pose startPose) {
        return endPose;
    }
    
    void setFollowingPower(double power) {
        followingPower = power;
    }
    
    void whileFollowing(Runnable action) {
        whileFollowingAction = () -> {
            whileFollowingAction.run();
            action.run();
        };
    }
    
    public abstract Command build(Pose startPose, Follower follower);
}
