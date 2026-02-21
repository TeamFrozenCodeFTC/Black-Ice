package org.firstinspires.ftc.teamcode.blackice.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private static final double BRAKING_POWER = -0.2;
    
    private static final int DRIVE_TIME_MS = 1000;
    private final ElapsedTime timer = new ElapsedTime();
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private final List<BrakeRecord> brakeData = new ArrayList<>();
    private State state = State.START_MOVE;
    private int iteration = 0;
    
    private Vector startPosition;
    private double measuredVelocity;
    
    @Override
    public void init() {
        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        telemetryM.debug(
            "The robot will move forwards and backwards starting at max speed and " +
                "slowing down.");
        telemetryM.debug("Make sure you have enough room. Leave at least 4-5 feet.");
        telemetryM.debug("After stopping, kFriction and kBraking will be displayed.");
        telemetryM.debug("Make sure to turn the timer off.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }
    
    @Override
    public void start() {
        timer.reset();
        follower.update();
        follower.startTeleOpDrive(true);
    }
    
    @Override
    public void loop() {
        follower.update();
        
        if (gamepad1.b) {
            stopRobot();
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
                follower.setMaxPower(currentPower);
                follower.setTeleOpDrive(direction, 0, 0, true);
                
                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }
            
            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = follower.getVelocity().getMagnitude();
                    startPosition = follower.getPose().getAsVector();
                    state = State.APPLY_BRAKE;
                }
                break;
            }
            
            case APPLY_BRAKE: {
                follower.setTeleOpDrive(BRAKING_POWER * direction, 0, 0, true);
                
                timer.reset();
                state = State.WAIT_BRAKE_TIME;
                break;
            }
            
            case WAIT_BRAKE_TIME: {
                double t = timer.milliseconds();
                Pose currentPose = follower.getPose();
                double currentVelocity = follower.getVelocity().getMagnitude();
                
                brakeData.add(new BrakeRecord(t, currentPose, currentVelocity));
                
                if (follower.getVelocity().dot(new Vector(direction,
                                                          follower.getHeading())) <= 0) {
                    state = State.RECORD;
                }
                break;
            }
            
            case RECORD: {
                Vector endPosition = follower.getPose().getAsVector();
                double brakingDistance = endPosition.minus(startPosition).getMagnitude();
                
                velocityToBrakingDistance.add(
                    new double[]{measuredVelocity, brakingDistance});
                
                telemetryM.debug("Test " + iteration,
                                 String.format("v=%.3f  d=%.3f", measuredVelocity,
                                               brakingDistance));
                telemetryM.update(telemetry);
                
                iteration++;
                state = State.START_MOVE;
                
                break;
            }
            
            case DONE: {
                stopRobot();
                
                double[] coefficients = quadraticFit(velocityToBrakingDistance);
                
                telemetryM.debug("Tuning Complete");
                telemetryM.debug("Braking Profile:");
                telemetryM.debug("kQuadratic", coefficients[1]);
                telemetryM.debug("kLinear", coefficients[0]);
                telemetryM.update(telemetry);
                telemetryM.debug("Tuning Complete");
                telemetryM.debug("Braking Profile:");
                telemetryM.debug("kQuadraticFriction", coefficients[1]);
                telemetryM.debug("kLinearBraking", coefficients[0]);
                for (BrakeRecord record : brakeData) {
                    Pose p = record.pose;
                    telemetryM.debug(
                        String.format("t=%.0f ms, x=%.2f, y=%.2f, θ=%.2f, v=%.2f",
                                      record.timeMs, p.getX(), p.getY(),
                                      p.getHeading(),
                                      record.velocity));
                }
                telemetryM.update();
                break;
            }
        }
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
        double powerApplied;
        
        BrakeRecord(double timeMs, Pose pose, double velocity) {
            this.timeMs = timeMs;
            this.pose = pose;
            this.velocity = velocity;
        }
    }
}
