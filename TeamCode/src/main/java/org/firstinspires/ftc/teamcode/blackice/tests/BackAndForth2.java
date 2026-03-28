package org.firstinspires.ftc.teamcode.blackice.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.FollowPathCommand;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class BackAndForth2 extends OpMode {
    Follower follower;
    
    Pose targetPose = new Pose(48, 0, 0);
    Pose startingPose = new Pose(0, 0, 0);
    
    AutoRoutine autoRoutine;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        follower.setTelemetry(telemetry);
        
        autoRoutine = follower.autoBuilder(startingPose)
            .lineTo(targetPose)
            .stop()
            .lineTo(startingPose)
            .stop()
            .lineTo(targetPose)
            .stop()
            .lineTo(startingPose)
            .stop()
            .lineTo(targetPose)
            .stop()
            .lineTo(startingPose)
            .stop()
            .lineTo(targetPose)
            .stop()
            .lineTo(startingPose)
            .stop()
            .build();
    }
    
    @Override
    public void start() {
        autoRoutine.start();
    }
    
    @Override
    public void loop() {
        follower.update();
        
        follower.telemetry.addLine(autoRoutine.toString());
        follower.telemetry.update();
        
        autoRoutine.run();
    }
}
