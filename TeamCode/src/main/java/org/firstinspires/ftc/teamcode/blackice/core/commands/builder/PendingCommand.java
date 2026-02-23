package org.firstinspires.ftc.teamcode.blackice.core.commands.builder;

import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

interface PendingCommand {
    Command build(Pose currentPose, Follower follower);
    default Pose getEndPose(Pose startPose) {
        return startPose;
    }
}
