package org.firstinspires.ftc.teamcode.blackice.core;

import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathPoint;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

public class FollowPathCommand extends Command {
    public final PathGeometry pathGeometry;
    final HeadingInterpolator headingInterpolator;
    final Follower follower;
    final double followingPower;
    public Pose endPose;
    double lastTValue = 0;
    
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
        
        double normalError = closest.point.minus(position).dot(normal);
        //        double normalDerivative =
        //            (normalError - previousNormalError) / follower.deltaTime;
        //
        //        previousNormalError = normalError;
        //
        double normalPower =
            follower.positionalController.computeOutput(
                normalError,
                velocity.dot(normal)
            ) * follower.normalAuthority;
        
        double distanceToEnd;
        if (closest.distanceRemaining == 0) {
            distanceToEnd =
                pathGeometry.getEndPoint().minus(position).dot(tangent);
        } else {
            distanceToEnd = closest.distanceRemaining;
        }
        
        double tangentPower =
            follower.positionalController.computeOutput(
                distanceToEnd,
                velocity.dot(tangent)
            );
        
        double targetHeading = headingInterpolator.interpolate(closest);
        double headingPower =
            follower.computeHeadingCorrectionPower(targetHeading);
        
        double maxMagnitude = 1.0;
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
        
        follower.telemetry.addData("holdPower",
                                   follower.computeHoldPower(pathGeometry.getEndPoint()));
        follower.telemetry.addData("distanceToEnd", distanceToEnd);
        follower.telemetry.addData("position", position);
        follower.telemetry.addData("closestT", closest.tValue);
        follower.telemetry.addData("normalError", normalError);
        follower.telemetry.addData("normalPower", normalPower);
        follower.telemetry.addData("tangentPower", tangentPower);
        follower.telemetry.addData("headingPower", headingPower);
        follower.telemetry.addData("percentAlongPath", closest.percentAlongPath);
        follower.telemetry.addData("currentPose", follower.getCurrentPose());
        follower.telemetry.addData("endPose", endPose);
        follower.telemetry.addData("drivePower", drivePower);
        follower.telemetry.update();
    }
    
    public double allocatePower(double requested, double budget) {
        return Math.copySign(
            Math.min(Math.abs(requested), budget),
            requested
        );
    }
    
    @Override
    public boolean isFinished() {
        return follower.isWithinBraking(endPose);
    }
}
