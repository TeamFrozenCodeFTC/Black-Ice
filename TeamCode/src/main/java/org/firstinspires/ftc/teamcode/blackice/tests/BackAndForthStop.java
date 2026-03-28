//package org.firstinspires.ftc.teamcode.blackice.tests;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
//import org.firstinspires.ftc.teamcode.blackice.core.Follower;
//import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
//
//@Config
//@Autonomous
//public class BackAndForthStop extends OpMode {
//    Follower follower;
//
//    public static double power = 1;
//    public static double linearTerm = 0.13692;
//    public static double quadTerm = 0;
//    public static double distance = 48;
//    public static double kP = 0.5;
//
//    Pose targetPose = new Pose(48, 0, 0);
//    Pose startingPose = new Pose(0, 0, 0);
//
//    int state = 0;
//
//    @Override
//    public void init() {
//        follower = FollowerConstants.createFollower(hardwareMap);
//        follower.setTelemetry(telemetry);
//    }
//
//    @Override
//    public void loop() {
//        follower.positionalController.kBraking = linearTerm;
//        follower.positionalController.kFriction = quadTerm;
//        follower.positionalController.kP = kP;
//
//        switch (state) {
//            case 0:
//                follower.holdPose(new Pose(distance, 0, 0), power);
//                if (follower.isStoppedAt(new Pose(distance, 0, 0))) {
//                    state++;
//                }
//                break;
//            case 1:
//                follower.holdPose(startingPose, power);
//                if (follower.isStoppedAt(startingPose)) {
//                    state--;
//                }
//                break;
//        }
//
//        telemetry.addData("position", follower.localizer.getPose());
//        telemetry.addData("state", state);
//        telemetry.update();
//
//        follower.update();
//    }
//}
