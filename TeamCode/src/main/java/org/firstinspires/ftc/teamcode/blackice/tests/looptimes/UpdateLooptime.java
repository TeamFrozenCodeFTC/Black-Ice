package org.firstinspires.ftc.teamcode.blackice.tests.looptimes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;

@Autonomous
public class UpdateLooptime extends OpMode {
    Follower follower;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
    }
    
    @Override
    public void loop() {
        
        follower.update();
        
        telemetry.addData("deltaTime", follower.getDeltaTime());
        telemetry.update();
    }
}
