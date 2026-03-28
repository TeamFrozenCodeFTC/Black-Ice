package org.firstinspires.ftc.teamcode.blackice.core;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathPoint;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

public class FollowPathCommand implements Command {
    public final PathGeometry pathGeometry;
    final HeadingInterpolator headingInterpolator;
    final Follower follower;
    final double followingPower;
    public Pose endPose;
    double lastTValue = 0;
    boolean isBraking = false;
    
    public FollowPathCommand(PathGeometry pathGeometry,
                             HeadingInterpolator headingInterpolator,
                             Follower follower,
                             double followingPower) {
        this.pathGeometry = pathGeometry;
        this.headingInterpolator = headingInterpolator;
        this.follower = follower;
        this.followingPower = followingPower;
        
        endPose = Pose.fromRadians(
            pathGeometry.getEndPoint(),
            headingInterpolator.interpolate(pathGeometry.getEndPathPoint())
        );
    }
    
    @Override
    public void start() {
        lastTValue = 0;
        isBraking = false;
    }
    
    @Override
    public void update() {
        Vector position = follower.getPosition();
        
        PathPoint closest =
            pathGeometry.computeClosestPathPointTo(position, lastTValue);
        lastTValue = closest.tValue;
        
        Vector tangent = Vector.fromPolar(1, closest.tangent);
        Vector normal = tangent.perpendicularLeft();
        
        Vector velocity = follower.getVelocity();
        double normalVelocity = velocity.dot(normal);
        
        double targetHeading = headingInterpolator.interpolate(closest);
        double headingPower =
            follower.computeHeadingCorrectionPower(targetHeading);
        
        double normalError = closest.point.minus(position).dot(normal);
        //        double normalDerivative =
        //            (normalError - previousNormalError) / follower.deltaTime;
        //
        //        previousNormalError = normalError;
        //
        double normalPower = follower.positionalController.computeOutput(normalError,
                                                                         normalVelocity);
        
        double distanceToEnd = getDistanceToEnd(closest, position, tangent);
        
        double tangentialVelocity = velocity.dot(tangent);
        
        double tangentPower = follower.positionalController.computeOutput(distanceToEnd
            , tangentialVelocity);
        isBraking = tangentPower <= 1;
        
        
        tangentPower *= Math.cos(Range.clip(Math.abs(normalPower) * 3 * Math.PI / 2, 0,
                                  Math.PI/2));
        
        double maxMagnitude = followingPower;
        double normalUsed = allocatePower(normalPower, maxMagnitude);
        double remaining =
            Math.sqrt(
                Math.max(
                    0.0,
                    maxMagnitude * maxMagnitude - normalUsed * normalUsed
                )
            );
        double headingUsed = allocatePower(headingPower, remaining);
        remaining =
            Math.sqrt(
                Math.max(
                    0.0,
                    remaining * remaining - headingUsed * headingUsed
                )
            );
        double tangentUsed =
            allocatePower(tangentPower, Math.min(followingPower, remaining));
        
        Vector drivePower =
            normal.times(normalUsed)
                .plus(tangent.times(tangentUsed));
        
        follower.followFieldVector(drivePower, headingPower);
        
        if (follower.telemetry == null) {
            return;
        }
        
        follower.telemetry.addData("distanceToEnd", distanceToEnd);
        follower.telemetry.addData("normalVelocity", normalVelocity);
        follower.telemetry.addData("tangentVelocity", tangentialVelocity);
        //follower.telemetry.addData("brakingDistance", initialBrakingDistance);
        follower.telemetry.addData("position", position);
        follower.telemetry.addData("closestT", closest.tValue);
        follower.telemetry.addData("normalError", normalError);
        follower.telemetry.addData("normalPower", normalPower);
        follower.telemetry.addData("tangentPower", tangentPower);
       // follower.telemetry.addData("normalControllerOutput", normalControllerOutput);
        follower.telemetry.addData("headingPower", headingPower);
        follower.telemetry.addData("percentAlongPath", closest.percentAlongPath);
        follower.telemetry.addData("currentPose", follower.getCurrentPose());
        follower.telemetry.addData("endPose", endPose);
        follower.telemetry.addData("drivePower", drivePower);
        follower.telemetry.update();
    }
    
    private double getDistanceToEnd(PathPoint closest, Vector position, Vector tangent) {
        double distanceToEnd;
        if (closest.distanceRemaining == 0) {
            distanceToEnd =
                pathGeometry.getEndPoint().minus(position).dot(tangent);
        } else {
            distanceToEnd = closest.distanceRemaining;
        }
        return distanceToEnd;
    }
    
    public double allocatePower(double requested, double budget) {
        return Math.copySign(
            Math.min(Math.abs(requested), budget),
            requested
        );
    }
    
    @Override
    public boolean isFinished() {
        return isBraking;
    }
    
    public String getName() {
        return "Path{" + endPose.toString() + ",lastTValue=" + lastTValue + "," +
            "isFinished=" + isFinished() + "}";
    }
}
