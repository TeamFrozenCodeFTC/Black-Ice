package org.firstinspires.ftc.teamcode.blackice.core.commands.builder;

import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.BezierGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.LineGeometry;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class AutoBuilder {
    private final AutoRoutine routine;
    private final Follower follower;
    
    private final Pose startPose;
    // +alliance Color
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
    
    private PendingMovement requireMovement() {
        if (pendingCommand == null || !(pendingCommand instanceof PendingMovement)) {
            throw new IllegalStateException("Last step is not a movement.");
        }
        
        return (PendingMovement) pendingCommand;
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
        pendingCommand = (PendingCommand) path;
        pendingCommands.add(pendingCommand);
        lastEndPose = path.endPose;
        return this;
    }
    
    public AutoBuilder lineTo(Pose target) {
        return addPath(new PendingPath(
            startPosition -> new LineGeometry(startPosition, target.getPosition()),
            target));
    }
    
    public AutoBuilder lineTo(Pose target, double timeout) {
        return this.lineTo(target).withTimeout(timeout);
    }
    
    public AutoBuilder curveTo(Pose controlPoint, Pose endPoint) {
        return addPath(new PendingPath(
            startPosition -> new BezierGeometry(startPosition, controlPoint.getPosition(),
                                                endPoint.getPosition()), endPoint));
    }
    
    /**
     * Holds the end pose after the path is completed until the condition is met.
     */
    public AutoBuilder holdUntil(Command condition) {
        return holdPose(lastEndPose).untilAllFinish(condition);
    }
    
    public AutoBuilder holdPose(Pose target) {
        return addPath(new PendingHoldPose(target));
    }
    
    /**
     * Continuously calls the given action while following the current path.
     */
    public AutoBuilder whileFollowing(Runnable action) {
        requireMovement().whileFollowing(action);
        return this;
    }
    
    public AutoBuilder withFollowingPower(double power) {
        requireMovement().setFollowingPower(power);
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
            return built.untilAllFinish(
                () -> follower.isStoppedAt(pendingCommand.getEndPose(currentPose1)));
        };
        return this;
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
    
    private PendingCommand wrapLastStepWith(
        java.util.function.Function<Command, Command> wrapper) {
        if (pendingCommand == null) return null;
        
        PendingCommand prev = pendingCommand;
        return (currentPose, follower) -> {
            Command built = prev.build(currentPose, follower);
            return wrapper.apply(built);
        };
    }
    
}
