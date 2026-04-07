package org.firstinspires.ftc.teamcode.blackice.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;
import org.firstinspires.ftc.teamcode.utils.ExecutorRegistry;

import java.util.Arrays;

@TeleOp
public class TeleOp2 extends OpMode {
    Robot robot;
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.follower.setTelemetry(telemetry);
    }
    
    double x = 0;
    double y = 0;
    double heading = 0;
    
    double mult = 150;

    @Override
    public void loop() {
        robot.follower.update();

        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            double predictedStoppingX =
                robot.follower.positionalController.getPredictedBrakingDisplacement(robot.follower.getVelocity().getX());
            x =
                robot.follower.getPosition().getX()
                    + predictedStoppingX
                    + mult * gamepad1.left_stick_x * robot.follower.getDeltaTime();
        }
        
        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            double predictedStoppingY =
                robot.follower.positionalController.getPredictedBrakingDisplacement(robot.follower.getVelocity().getY());
            y =
                robot.follower.getPosition().getY()
                    + predictedStoppingY
                    - mult * gamepad1.left_stick_y * robot.follower.getDeltaTime();
        }
        if (Math.abs(gamepad1.right_stick_x) > 0) {
            heading =
                robot.follower.getHeading()
                    - 30 * gamepad1.right_stick_x * robot.follower.getDeltaTime();
        } // todo prio heading
        
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", heading);
        telemetry.update();
        
        robot.follower.holdPose(Pose.fromRadians(new Vector(x, y), heading));
    }
    
    @Override
    public void stop() {
        ExecutorRegistry.shutdownAll();
    }
}
