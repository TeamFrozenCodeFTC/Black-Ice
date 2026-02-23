package org.firstinspires.ftc.teamcode.blackice.tuning;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

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
public class PredictiveBrakingTuner extends OpMode {
    private static final double[] TEST_POWERS =
        {1, 1, 1, 0.9, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    
    private static double[] generateSpread(
        int count,
        double max,
        double min,
        double flatFraction // 0.0–1.0 portion that stays at max
    ) {
        double[] values = new double[count];
        
        int flatCount = (int) (count * flatFraction);
        int rampCount = count - flatCount;
        
        for (int i = 0; i < count; i++) {
            if (i < flatCount) {
                values[i] = max;
            } else {
                double t = (double)(i - flatCount) / (rampCount - 1);
                values[i] = max - t * (max - min);
            }
        }
        
        return values;
    }
    
    private static double[] biasedGradient(
        int count,
        double max,
        double min,
        double bias // >1 = more high values (try 2–3)
    ) {
        if (count < 2) return new double[]{max};
        
        double[] values = new double[count];
        
        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);
            
            // Ease-out curve (clusters near max)
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
    }

    @Override
    public void init_loop() {
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
        return follower.getVelocity().dot(new Vector(1, follower.getHeading()));
    }
    
    public double getVelocityMagnitude() {
        return follower.getVelocity().computeMagnitude();
    }
    
    private static final Vector FORWARD = new Vector(1, 0);
    
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
        
        double direction = (iteration % 2 == 0) ? 1 : -1;
        
        switch (state) {
            case START_MOVE: {
                if (iteration >= TEST_POWERS.length) {
                    state = State.DONE;
                    break;
                }
                
                double currentPower = TEST_POWERS[iteration];
                follower.drivetrain.followVector(FORWARD.times(currentPower * direction), 0); // heading correction?
                
                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }
            
            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = getForwardVelocity();
                    startPosition = follower.getPosition();
                    state = State.APPLY_BRAKE;
                }
                break;
            }
            
            case APPLY_BRAKE: {
                follower.drivetrain.followVector(
                    FORWARD.times(-follower.positionalController.maximumBrakingPower * direction), 0); // or maybe field relative braking?
                
                timer.reset();
                state = State.WAIT_BRAKE_TIME;
                break;
            }
            
            case WAIT_BRAKE_TIME: {
                double t = timer.milliseconds();
                Pose currentPose = follower.getCurrentPose();
                double currentVelocity = getForwardVelocity();
                double velocityMagnitude = getVelocityMagnitude();
                double voltage = follower.getVoltage();
                double appliedVoltage = voltage * TEST_POWERS[iteration];
                
                brakeData.add(new BrakeRecord(t, currentPose, currentVelocity,
                                              appliedVoltage, velocityMagnitude));
                
                if (currentVelocity <= 0) {
                    state = State.RECORD;
                }
                break;
            }
            
            case RECORD: {
                Vector endPosition = follower.getPosition();
                double brakingDistance = endPosition.minus(startPosition).computeMagnitude();
                
                velocityToBrakingDistance.add(
                    new double[]{measuredVelocity, brakingDistance});
                
                follower.telemetry.addData("Test " + iteration,
                                 String.format("v=%.3f  d=%.3f", measuredVelocity,
                                               brakingDistance));
                follower.telemetry.update();
                
                iteration++;
                state = State.START_MOVE;
                
                break;
            }
            
            case DONE: {
                follower.drivetrain.zeroPower();
                follower.drivetrain.zeroPowerBrakeMode();
                
                double[] coefficients = quadraticFit(velocityToBrakingDistance);
                
                follower.telemetry.addLine("Tuning Complete");
                follower.telemetry.addLine("Braking Profile:");
                follower.telemetry.addData("kQuadratic", coefficients[1]);
                follower.telemetry.addData("kLinear", coefficients[0]);
                follower.telemetry.update();
                follower.telemetry.addLine("Tuning Complete");
                follower.telemetry.addLine("Braking Profile:");
                follower.telemetry.addData("kQuadraticFriction", coefficients[1]);
                follower.telemetry.addData("kLinearBraking", coefficients[0]);
                
                for (BrakeRecord record : brakeData) {
                    follower.telemetry.addLine(record.toString());
                }
                follower.telemetry.update();
                break;
            }
        }
    }

    private double[] quadraticFit(List<double[]> velocityToBrakingDistance) {
        // Least-squares fit for d = a*v^2 + b*v
        double sumV = 0;
        double sumV2 = 0;
        double sumV3 = 0;
        double sumV4 = 0;
        
        double sumD = 0;
        double sumVD = 0;
        double sumV2D = 0;
        
        int n = velocityToBrakingDistance.size();
        if (n == 0) return new double[]{0, 0};
        
        for (double[] entry : velocityToBrakingDistance) {
            double v = entry[0];
            double d = entry[1];
            
            double v2 = v * v;
            
            sumV += v;
            sumV2 += v2;
            sumV3 += v2 * v;
            sumV4 += v2 * v2;
            
            sumD += d;
            sumVD += v * d;
            sumV2D += v2 * d;
        }
        
        // Solve normal equations:
        // [ Σv^2   Σv^3 ] [ b ] = [ Σv*d  ]
        // [ Σv^3   Σv^4 ] [ a ]   [ Σv^2*d ]
        double A11 = sumV2;
        double A12 = sumV3;
        double A21 = sumV3;
        double A22 = sumV4;
        
        double B1 = sumVD;
        double B2 = sumV2D;
        
        double det = A11 * A22 - A12 * A21;
        if (Math.abs(det) < 1e-9) return new double[]{0, 0};
        
        double b = (B1 * A22 - B2 * A12) / det;  // linear term
        double a = (A11 * B2 - A21 * B1) / det;  // quadratic term
        
        return new double[]{b, a};
    }
    
    private enum State {
        START_MOVE,
        WAIT_DRIVE_TIME,
        APPLY_BRAKE,
        WAIT_BRAKE_TIME,
        RECORD,
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
