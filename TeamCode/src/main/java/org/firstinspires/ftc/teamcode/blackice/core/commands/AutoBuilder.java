package org.firstinspires.ftc.teamcode.blackice.core.commands;

import org.firstinspires.ftc.teamcode.blackice.core.FollowPathCommand;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.BezierGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.LineGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

import java.util.ArrayList;
import java.util.List;

public class AutoBuilder {
    private final AutoRoutine routine;
    private final Follower follower;
    
    private final Pose startPose;
    private final List<PendingCommand> pendingCommands = new ArrayList<>();
    private PendingCommand pendingCommand;
    private Pose lastEndPose;
    
    public AutoBuilder(Pose startPose, Follower follower) {
        this.startPose = startPose;
        this.routine = new AutoRoutine(startPose);
        this.follower = follower;
    }
    
    public AutoBuilder addRoutine(AutoBuilder addedRoutine) {
        flushPending();
        
        pendingCommands.addAll(addedRoutine.pendingCommands);
        
        return this;
    }
    
    private PendingPath requirePath() {
        if (pendingCommand == null || !(pendingCommand instanceof PendingPath)) {
            throw new IllegalStateException("Last step is not a path.");
        }
        
        return (PendingPath) pendingCommand;
    }
    
    public AutoRoutine build() {
        if (startPose == null) {
            throw new IllegalStateException(
                "Starting pose must be set before building the routine.");
        }
        
        return build(startPose);
    }
    
    public AutoRoutine build(Pose startPose) {
        flushPending();
        
        Pose currentPose = startPose;
        
        for (PendingCommand path : pendingCommands) {
            routine.add(path.build(currentPose, follower));
            currentPose = path.getEndPose(currentPose);
        }
        
        routine.add(Command.singleAction(follower::stop));
        return routine;
    }
    
    private void flushPending() {
        pendingCommand = null;
    }
    
    private AutoBuilder addPath(PendingMovement path) {
        flushPending();
        pendingCommand = path;
        pendingCommands.add(pendingCommand);
        lastEndPose = path.endPose;
        return this;
    }
    
    public AutoBuilder lineTo(Pose target) {
        return addPath(new PendingPath(
            startPosition -> new LineGeometry(
                startPosition,
                target.getPosition()
            ),
            target
        ));
    }
    
    public AutoBuilder lineTo(Pose target, double timeout) {
        return this.lineTo(target).withTimeout(timeout);
    }
    
    public AutoBuilder curveTo(Pose controlPoint, Pose endPoint) {
        return addPath(new PendingPath(
            startPosition -> new BezierGeometry(
                startPosition,
                controlPoint.getPosition(),
                endPoint.getPosition()
            ),
            endPoint
        ));
    }
    
    /**
     * Holds the end pose after the path is completed until the condition is met.
     */
    public AutoBuilder holdUntil(Command condition) {
        return holdPose(lastEndPose)
            .untilAllFinish(condition);
    }
    
    public AutoBuilder holdPose(Pose target) {
        return addPath(new PendingHoldPose(target));
    }
    
    /**
     * Continuously calls the given action while following the current path.
     */
    public AutoBuilder whileFollowing(Runnable action) {
        requirePath().whileFollowing(action);
        return this;
    }
    
    public AutoBuilder withFollowingPower(double power) {
        requirePath().setFollowingPower(power);
        return this;
    }
    
    public AutoBuilder waitSeconds(double seconds) {
        addCommand(Command.waitSeconds(seconds));
        return this;
    }
    
    public AutoBuilder stop() {
        if (pendingCommand == null) return null;
        
        PendingCommand prev = pendingCommand;
        pendingCommand = (currentPose1, follower) -> {
            Command built = prev.build(currentPose1, follower);
            return built.untilAllFinish(new Command() {
                @Override
                public boolean isFinished() {
                    return follower.isStoppedAt(pendingCommand.getEndPose(currentPose1));
                }
            });
        };
    }
    
    public AutoBuilder addCommand(Runnable action) {
        return addCommand(Command.singleAction(action));
    }
    
    public AutoBuilder addCommand(Command command) {
        return addCommand((currentPose, follower) -> command);
    }
    
    AutoBuilder addCommand(PendingCommand step) {
        flushPending();
        pendingCommands.add(step);
        return this;
    }
    
    public AutoBuilder withLinearHeadingInterpolation() {
        requirePath().setHeadingInterpolator(HeadingInterpolator::linear);
        return this;
    }
    
    private PendingCommand wrapLastStepWith(
        java.util.function.Function<Command, Command> wrapper) {
        if (pendingCommand == null) return null;
        
        PendingCommand prev = pendingCommand;
        return (currentPose, follower) -> {
            Command built = prev.build(currentPose, follower);
            return wrapper.apply(built);
        };
    }
    
    public AutoBuilder untilEitherFinishes(Command other) {
        pendingCommand = wrapLastStepWith(cmd -> cmd.untilEitherFinishes(other));
        return this;
    }
    
    public AutoBuilder withTimeout(double seconds) {
        if (pendingCommand instanceof PendingPath) {
            ((PendingPath) pendingCommand).setTimeout(seconds);
        } else {
            pendingCommand = wrapLastStepWith(cmd -> cmd.withTimeout(seconds));
        }
        return this;
    }
    
    public AutoBuilder untilAllFinish(Command other) {
        pendingCommand = wrapLastStepWith(cmd -> cmd.untilAllFinish(other));
        return this;
    }
    
    private interface PendingCommand {
        Command build(Pose currentPose, Follower follower);
        
        default Pose getEndPose(Pose startPose) {
            return startPose;
        }
    }
    
    private static abstract class PendingMovement implements PendingCommand {
        Pose endPose;
        double followingPower = 1;
        Runnable whileFollowingAction = () -> {};
        
        @Override
        public Pose getEndPose(Pose startPose) {
            return endPose;
        }
        
        void setFollowingPower(double power) {
            followingPower = power;
        }
        
        void whileFollowing(Runnable action) {
            whileFollowingAction = () -> {
                whileFollowingAction.run();
                action.run();
            };
        }
        
        public abstract Command build(Pose startPose, Follower follower);
    }
    
    private static class PendingPath extends PendingMovement {
        final GeometryConstructor geometryConstructor;
        HeadingConstructor headingConstructor;
        Double timeout = null;
        
        PendingPath(GeometryConstructor constructor, Pose endPose) {
            this.geometryConstructor = constructor;
            this.headingConstructor = HeadingInterpolator::linear;
            this.endPose = endPose;
        }
        
        @Override
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
                timeout =
                    (pathGeometry.length() / follower.drivetrain.getMaxVelocity()) *
                        follower.estimatedPathTimeoutMultiplier +
                        follower.minimumPathTimeout;
            }
            
            Command cmd = new FollowPathCommand(
                pathGeometry,
                headingConstructor.create(startPose.getHeading(), endPose.getHeading()),
                follower,
                followingPower
            );
            
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
    
    private static class PendingHoldPose extends PendingMovement {
        double followingPower = 1;
        
        PendingHoldPose(Pose endPose) {
            this.endPose = endPose;
        }
        
        @Override
        public Command build(Pose startPose, Follower follower) {
            return new Command() {
                @Override
                public void update() {
                    follower.holdPose(endPose, followingPower);
                }
                
                @Override
                public boolean isFinished() {
                    return follower.isStoppedAt(endPose);
                }
            };
        }
    }
}
