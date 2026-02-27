package org.firstinspires.ftc.teamcode.blackice.tests.looptimes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class HoldPoseLooptime extends OpMode {
    Follower follower;
    
    Pose startingPose = new Pose(0, 0, 0);
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
    }
    
    @Override
    public void loop() {
        follower.update();
        
        follower.holdPose(startingPose);
        
        telemetry.addData("deltaTime", follower.getDeltaTime());
        telemetry.update();
    }
}
