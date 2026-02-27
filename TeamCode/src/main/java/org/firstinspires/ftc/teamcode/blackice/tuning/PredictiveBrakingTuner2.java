package org.firstinspires.ftc.teamcode.blackice.tuning;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

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
public class PredictiveBrakingTuner2 extends OpMode {
    private static final double[] TEST_POWERS = biasedGradient(30, 1.0, 0.2, 2);
    
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

    private static final int DRIVE_TIME_MS = 1000;
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
    
    private void startMovement(int direction, double power) {
        follower.drivetrain.followVector(FORWARD.times(power * direction), 0);
        timer.reset();
    }
    
    private void startBraking(int direction) {
        follower.drivetrain.followVector(
            FORWARD.times(-follower.positionalController.maximumBrakingPower * direction), 0);
        timer.reset();
    }
    
    private BrakeRecord recordBrakeData(double velocity) {
        double t = timer.milliseconds();
        Pose currentPose = follower.getCurrentPose();
        double velocityMagnitude = getVelocityMagnitude();
        double voltage = follower.getVoltage();
        double appliedVoltage = -voltage * Math.abs(TEST_POWERS[iteration]) * 0.2;
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
                
                startMovement(getDirection(), TEST_POWERS[iteration]);
                state = State.WAIT_DRIVE_TIME;
                break;
            }
            
            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    startPosition = follower.getPosition();
                    measuredVelocity = getForwardVelocity();
                    
                    brakeData.add(recordBrakeData(measuredVelocity));
                    startBraking(getDirection());
                    state = State.WAIT_BRAKE_TIME;
                }
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
                follower.telemetry.addLine("Braking Profile:");
                follower.telemetry.addData("kQuadraticFriction", coefficients[1]);
                follower.telemetry.addData("kLinearBraking", coefficients[0]);
                
                for (BrakeRecord record : brakeData) {
                    follower.telemetry.addLine(record.toString());
                }
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
            startMovement(getDirection(), TEST_POWERS[iteration]);
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
