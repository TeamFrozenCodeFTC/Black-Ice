package org.firstinspires.ftc.teamcode.blackice.tuning;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;
import org.firstinspires.ftc.teamcode.utils.Regression;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the Predictive Braking Tuner. It runs the robot forward and backward at various
 * power levels, recording the robot’s velocity and position immediately before braking.
 * The motors are then set to a reverse power, which represents the fastest theoretical
 * braking the robot can achieve. Once the robot comes to a complete stop, the tuner
 * measures the stopping distance. Using the collected data, it generates a
 * velocity-vs-stopping-distance graph and fits a quadratic curve to model the braking
 * behavior.
 *
 * @author Jacob Ophoven - 18535 Frozen Code
 * @version 1.0, 12/26/2025
 */
@Autonomous
@Config
public class PredictiveBrakingTuner2 extends OpMode {
    private static double[] TEST_POWERS;
    
    public static int trials = 30;
    public static double max = 0.4;
    public static double min = 0.1;
    public static double bias = 1;
    
    public static double brakingPower = 0.2;
    
    private static double[] biasedGradient(
        int count,
        double max,
        double min,
        double bias
    ) {
        if (count < 2) return new double[]{max};
        
        double[] values = new double[count];
        
        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);
            
            double curved = 1 - Math.pow(t, bias);
            
            values[i] = min + curved * (max - min);
        }
        
        return values;
    }

    public static int DRIVE_TIME_MS = 1500;
    private final ElapsedTime timer = new ElapsedTime();
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private final List<BrakeRecord> brakeData = new ArrayList<>();
    private State state = State.START_MOVE;
    private int iteration = 0;
    
    private Vector startPosition;
    private double measuredVelocity;
    
    private Follower follower;
    
    @Override
    public void init() {
        TEST_POWERS = biasedGradient(trials, max, min, bias);
        
        follower = FollowerConstants.createFollower(hardwareMap);
        
        follower.setTelemetry(telemetry);

        follower.telemetry.addLine(
            "The robot will move forwards and backwards starting at max speed and " +
                "slowing down.");
        follower.telemetry.addLine("Make sure you have enough room. Leave at least 4-5 feet.");
        follower.telemetry.addLine("After stopping, kFriction and kBraking will be displayed.");
        follower.telemetry.addLine("Make sure to turn the timer off.");
        follower.telemetry.addLine("Press B on game pad 1 to stop.");
        follower.telemetry.update();
        follower.update();
    }
    
    @Override
    public void start() {
        timer.reset();
        follower.update();
    }

    public double getForwardVelocity() {
        return follower.getVelocity().dot(new Vector(getDirection(), follower.getHeading()));
    }
    
    public double getVelocityMagnitude() {
        return follower.getVelocity().computeMagnitude();
    }
    
    private static final Vector FORWARD = new Vector(1, 0);
    
    private int getDirection() {
        return (iteration % 2 == 0) ? 1 : -1;
    }
    
    private void drive(int direction, double power) {
        follower.drivetrain.followVector(FORWARD.times(power * direction),
                                         follower.computeHeadingCorrectionPower(0));
    }
    
    private void startBraking(int direction) {
        follower.drivetrain.followVector(
            FORWARD.times(-brakingPower * direction), 0);
        timer.reset();
    }
    
    private BrakeRecord recordBrakeData(double velocity) {
        double t = timer.milliseconds();
        Pose currentPose = follower.getCurrentPose();
        double velocityMagnitude = getVelocityMagnitude();
        double voltage = follower.getVoltage();
        double appliedVoltage = voltage * -brakingPower;
        return new BrakeRecord(t, currentPose, velocity, appliedVoltage, velocityMagnitude);
    }
    
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();
        
        if (gamepad1.b) {
            follower.drivetrain.zeroPower();
            follower.drivetrain.zeroPowerBrakeMode();
            requestOpModeStop();
            return;
        }
        
        switch (state) {
            case START_MOVE: {
                if (iteration >= TEST_POWERS.length) {
                    state = State.DONE;
                    break;
                }
                
                drive(getDirection(), TEST_POWERS[iteration]);
                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }
            
            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    timer.reset();
                    
                    startPosition =
                        follower.getPosition().plus(follower.getVelocity().times(follower.getDeltaTime()));
                    measuredVelocity =
                        getVelocityMagnitude();
                     
                    brakeData.add(recordBrakeData(measuredVelocity));
                    startBraking(getDirection());
                    state = State.WAIT_BRAKE_TIME;
                    break;
                }
                drive(getDirection(), TEST_POWERS[iteration]);
                break;
            }
            
            case WAIT_BRAKE_TIME: {
                double currentVelocity = getForwardVelocity();
                brakeData.add(recordBrakeData(currentVelocity));
                
                if (currentVelocity <= 0) {
                    finishIteration();
                }
                break;
            }
            
            case DONE: {
                follower.drivetrain.zeroPower();
                follower.drivetrain.zeroPowerBrakeMode();
                
                double[] coefficients = Regression.quadraticFit(velocityToBrakingDistance);
                
                follower.telemetry.addLine("Tuning Complete");
                follower.telemetry.addLine("Braking Profile (initial):");
                follower.telemetry.addData("quadratic", coefficients[1]);
                follower.telemetry.addData("linear", coefficients[0]);
                
                for (BrakeRecord record : brakeData) {
                    follower.telemetry.addLine(record.toString());
                }
                follower.telemetry.update();
                
//                List<double[]> remainingData = new ArrayList<>();
//                for (int i = 0; i < brakeData.size(); ) {
//                    int endIdx = i;
//                    while (endIdx < brakeData.size() && brakeData.get(endIdx).velocity > 0) {
//                        endIdx++;
//                    }
//
//                    for (int j = i + 1; j < endIdx; j++) {
//                        double remainingDistance =
//                            brakeData.get(endIdx - 1).pose.getPosition().minus(brakeData.get(j).pose.getPosition()).computeMagnitude();
//                        remainingData.add(new double[]{brakeData.get(j).velocity, remainingDistance});
//                    }
//
//                    i = endIdx; // move to next run
//                }
                List<double[]> remainingData = new ArrayList<>();
                
                int runStart = 0;
                while (runStart < brakeData.size()) {
                    // Find the end of this run (velocity > 0)
                    int runEnd = runStart;
                    while (runEnd < brakeData.size() && brakeData.get(runEnd).velocity > 0) {
                        runEnd++;
                    }
                    
                    if (runEnd - runStart < 2) {
                        // not enough points in this run, skip
                        runStart = runEnd + 1;
                        continue;
                    }
                    
                    // Only compute remaining distance relative to the last point of this run once
                    Pose lastPose = brakeData.get(runEnd - 1).pose;
                    for (int j = runStart; j < runEnd - 1; j++) {
                        double remainingDistance = lastPose.getPosition()
                            .minus(brakeData.get(j).pose.getPosition())
                            .computeMagnitude();
                        remainingData.add(new double[]{brakeData.get(j).velocity, remainingDistance});
                    }
                    
                    // Move to next run
                    runStart = runEnd + 1;
                }
                
                follower.telemetry.addData("remainingData size", remainingData.size());
                follower.telemetry.addLine("waitingOnRegression");
                follower.telemetry.update();
                
                double[] remainingCoefficients = Regression.quadraticFit(remainingData);
                
                follower.telemetry.addLine("Braking Profile (non-initial):");
                follower.telemetry.addData("quadratic", remainingCoefficients[1]);
                follower.telemetry.addData("linear", remainingCoefficients[0]);
                follower.telemetry.update();
                
                requestOpModeStop();
                break;
            }
        }
    }
    
    private void finishIteration() {
        Vector endPosition = follower.getPosition();
        double brakingDistance = endPosition.minus(startPosition).computeMagnitude();
        
        velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});
        
        follower.telemetry.addData("Test " + iteration,
                                   String.format("v=%.3f  d=%.3f", measuredVelocity, brakingDistance));
        follower.telemetry.update();
        
        iteration++;
        
        if (iteration >= TEST_POWERS.length) {
            state = State.DONE;
        } else {
            drive(getDirection(), TEST_POWERS[iteration]);
            state = State.WAIT_DRIVE_TIME;
        }
    }

    private enum State {
        START_MOVE,
        WAIT_DRIVE_TIME,
        WAIT_BRAKE_TIME,
        DONE
    }
    
    private static class BrakeRecord {
        double timeMs;
        Pose pose;
        double velocity;
        double voltageApplied;
        double velocityMagnitude;
        
        BrakeRecord(double timeMs, Pose pose, double velocity, double voltageApplied, double velocityMagnitude) {
            this.timeMs = timeMs;
            this.pose = pose;
            this.velocity = velocity;
            this.voltageApplied = voltageApplied;
            this.velocityMagnitude = velocityMagnitude;
        }
        
        @SuppressLint("DefaultLocale")
        @NonNull
        public String toString() {
            return String.format("t=%.0f ms, x=%.2f, y=%.2f, θ=%.2f, vel=%.2f, velMag=%" +
                                     ".2f " +
                                     "voltageApplied=%.2f",
                                 timeMs, pose.getX(), pose.getY(), pose.getHeading(),
                                 velocity, velocityMagnitude, voltageApplied);
        }
    }
}


//Test 0: v=71.916 d=10.169
//Test 1: v=74.030 d=10.335
//Test 2: v=73.089 d=9.934
//Test 3: v=70.969 d=9.620
//Test 4: v=69.345 d=9.435
//Test 5: v=69.985 d=8.779
//Test 6: v=67.624 d=8.671
//Test 7: v=63.681 d=8.793
//Test 8: v=65.392 d=7.539
//Test 9: v=61.015 d=7.518
//Test 10: v=61.418 d=7.237
//Test 11: v=59.511 d=7.235
//Test 12: v=58.997 d=6.552
//Test 13: v=56.029 d=6.284
//Test 14: v=53.673 d=5.537
//Test 15: v=51.506 d=4.331
//Test 16: v=48.660 d=4.024
//Test 17: v=45.302 d=3.568
//Test 18: v=43.310 d=3.210
//Test 19: v=41.610 d=2.993
//Test 20: v=38.758 d=2.272
//Test 21: v=34.186 d=2.511
//Test 22: v=32.863 d=2.100
//Test 23: v=29.030 d=1.824
//Test 24: v=27.698 d=2.189
//Test 25: v=24.428 d=1.682
//Test 26: v=22.514 d=1.495
//Test 27: v=18.432 d=1.117
//Test 28: v=15.156 d=0.894
//Test 29: v=12.591 d=0.562
