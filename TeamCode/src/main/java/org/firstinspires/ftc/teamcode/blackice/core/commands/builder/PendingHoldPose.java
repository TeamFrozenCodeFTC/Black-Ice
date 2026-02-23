package org.firstinspires.ftc.teamcode.blackice.core.commands.builder;

import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

class PendingHoldPose extends PendingMovement {
    double followingPower = 1;
    
    PendingHoldPose(Pose endPose) {
        this.endPose = endPose;
    }
    
    @Override
    public Command build(Pose startPose, Follower follower) {
        return new Command() {
            @Override
            public void update() {
                follower.holdPose(endPose, followingPower);
            }
            
            @Override
            public boolean isFinished() {
                return follower.isStoppedAt(endPose);
            }
        };
    }
}
