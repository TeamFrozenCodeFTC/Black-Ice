package org.firstinspires.ftc.teamcode.blackice.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.FollowPathCommand;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class Maze extends OpMode {
    Follower follower;
    
    AutoRoutine autoRoutine;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        
        follower.setTelemetry(telemetry);
        
        autoRoutine = follower.autoBuilder(new Pose(0,0,0))
            .lineTo(new Pose(48, 0, 0))
            .lineTo(new Pose(48, 48, 0))
            .lineTo(new Pose(0, 48, 0))
            .lineTo(new Pose(0, 0, 0))
            .lineTo(new Pose(48, 0, 90))
            .lineTo(new Pose(48, 48, 90))
            .lineTo(new Pose(0, 48, 0))
            .lineTo(new Pose(0, 0, 0))
            .untilAllFinish(() -> false)
            .build();
    }
    
    @Override
    public void start() {
        autoRoutine.start();
    }
    
    @Override
    public void loop() {
        follower.update();
        
        autoRoutine.run();
        
        follower.telemetry.addLine(autoRoutine.toString());
        follower.telemetry.update();
    }
}
