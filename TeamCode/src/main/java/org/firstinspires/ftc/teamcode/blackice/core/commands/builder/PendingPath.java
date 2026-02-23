package org.firstinspires.ftc.teamcode.blackice.core.commands.builder;

import org.firstinspires.ftc.teamcode.blackice.core.FollowPathCommand;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

class PendingPath extends PendingMovement {
    final GeometryConstructor geometryConstructor;
    HeadingConstructor headingConstructor;
    Double timeout = null;
    
    PendingPath(GeometryConstructor constructor, Pose endPose) {
        this.geometryConstructor = constructor;
        this.headingConstructor = HeadingInterpolator::linear;
        this.endPose = endPose;
    }
    
    public void setHeadingInterpolator(HeadingConstructor interpolator) {
        this.headingConstructor = interpolator;
    }
    
    public void setTimeout(double seconds) {
        this.timeout = seconds;
    }
    
    @Override
    public Command build(Pose startPose, Follower follower) {
        PathGeometry pathGeometry = geometryConstructor.create(startPose.getPosition());
        
        if (timeout == null) {
            timeout = (pathGeometry.length() / follower.drivetrain.getMaxVelocity()) *
                follower.estimatedPathTimeoutMultiplier + follower.minimumPathTimeout;
        }
        
        Command cmd = new FollowPathCommand(pathGeometry, headingConstructor.create(
            startPose.getHeading(), endPose.getHeading()), follower, followingPower);
        
        if (timeout != null) {
            cmd = cmd.withTimeout(timeout);
        }
        
        return cmd;
    }
    
    public interface GeometryConstructor {
        PathGeometry create(Vector startPosition);
    }
    
    public interface HeadingConstructor {
        HeadingInterpolator create(double startPose, double endPose);
    }
}
