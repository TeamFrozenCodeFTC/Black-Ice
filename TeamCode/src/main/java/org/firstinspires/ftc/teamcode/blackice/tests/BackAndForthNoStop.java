package org.firstinspires.ftc.teamcode.blackice.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class BackAndForthNoStop extends OpMode {
    Follower follower;
    
    Pose targetPose = new Pose(48+24, 0, 0);
    Pose startingPose = new Pose(0, 0, 0);
    
    AutoRoutine autoRoutine;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        follower.setTelemetry(telemetry);
        
        follower.telemetry.addData("targetVelocity", 0);
        follower.telemetry.addData("currentVelocity", 0);
        follower.telemetry.update();
        
        autoRoutine = follower.autoBuilder(startingPose)
            .lineTo(targetPose)
            .lineTo(startingPose)
            .lineTo(targetPose)
            .lineTo(startingPose)
            .lineTo(targetPose)
            .lineTo(startingPose)
            .lineTo(targetPose)
            .lineTo(startingPose)
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
