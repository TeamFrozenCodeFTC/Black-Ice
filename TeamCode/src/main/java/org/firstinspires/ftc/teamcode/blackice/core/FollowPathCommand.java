package org.firstinspires.ftc.teamcode.blackice.core;

import com.qualcomm.robotcore.util.Range;

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
    boolean shouldBrake = false;
    boolean isHolding = false;
    
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
        shouldBrake = false;
    }
    
   // @Override
    public void update3() {
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
        double normalPower = follower.positionalController.computeOutput(normalError, normalVelocity);
        
        double distanceToEnd = getDistanceToEnd(closest, position, tangent);
        
        double tangentialVelocity = velocity.dot(tangent);
        
        double tangentPower = follower.positionalController.computeOutput(distanceToEnd
            , tangentialVelocity);
        shouldBrake = tangentPower <= 1;
        
        
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
    
    public void update2() {
        Vector position = follower.getPosition();
        
        PathPoint closest =
            pathGeometry.computeClosestPathPointTo(position, lastTValue);
        lastTValue = closest.tValue;
        
        Vector tangent = Vector.fromPolar(1, closest.tangent);
        Vector normal = tangent.perpendicularLeft();
        
        //
        
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
        shouldBrake = tangentPower <= 1;
        
        double radiansPerDistance = (Math.PI / 2) / follower.trackWidth;
        double estimatedError = normalPower / follower.positionalController.kP;
        double directionScale = cosineScale(estimatedError, radiansPerDistance);
        tangentPower *= directionScale;
        
        double headingErrorEstimate = headingPower / follower.headingController.kP;
        double headingScale = Math.cos(
            Math.min(Math.abs(headingErrorEstimate), Math.PI / 2)
        );
        tangentPower *= headingScale;
        
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
        double normalPower = follower.positionalController.computeOutput(normalError,
                                                                         normalVelocity);
        
        double distanceToEnd = getDistanceToEnd(closest, position, tangent);
        double tangentialVelocity = velocity.dot(tangent);

        double tangentPower = drive2(distanceToEnd, tangentialVelocity);
        //double tangentPower = pedro3Algo(distanceToEnd, tangentialVelocity);
            
        double radiansPerDistance = (Math.PI / 2) / follower.trackWidth;
        double estimatedError = normalPower / follower.positionalController.kP;
        double directionScale = cosineScale(estimatedError, radiansPerDistance);
        tangentPower *= directionScale;
        
        double headingErrorEstimate = headingPower / follower.headingController.kP;
        double headingScale = Math.cos(
            Math.min(Math.abs(headingErrorEstimate), Math.PI / 2)
        );
        tangentPower *= headingScale;

        follower.telemetry.addData("length", pathGeometry.length());
        follower.telemetry.addData("acceleration", follower.getAcceleration());
        follower.telemetry.addData("closestPoint", closest.point);
        follower.telemetry.addData("isHolding", isHolding);
        
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
        
        
        follower.telemetry.addData("isBraking", shouldBrake);
        follower.telemetry.addData("distanceToEnd", distanceToEnd);
        follower.telemetry.addData("normalVelocity", normalVelocity);
        follower.telemetry.addData("tangentVelocity", tangentialVelocity);
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
    
    public double cosineScale(double error, double scaleFactor) {
        double clamped = Math.min(Math.abs(error) * scaleFactor, Math.PI / 2);
        return Math.cos(clamped);
    }
    
    public double pedro3Algo(double distanceToEnd, double tangentialVelocity) {
        double eigenX = 0.0015;
        double eigenY = 0.0015;
        
        shouldBrake = true;

        double linBrakeX = 0.045;
        double linBrakeY = 0.045;

        double alpha = 1;

//        double tx = tangent.getX();
//        double ty = tangent.getY();
        double tx = 1;
        double ty = 0;

        // Ellipsoid projection: M * tangent
        double mtx = eigenX * tx;
        double mty = eigenY * ty;

        double k2 = alpha * (tx * mtx + ty * mty);
        double k1 = alpha * (tx * linBrakeX + ty * linBrakeY);

        double discrim = k1 * k1 + 4 * k2 * distanceToEnd;
        double targetVelocity =
            (-k1 + Math.signum(discrim) * Math.sqrt(Math.abs(discrim))) / (2 * k2);

        double velocityError = targetVelocity - tangentialVelocity;
        double tangentPower = velocityError * 0.02;// + targetVelocity * 0.015;
        return tangentPower;
    }
    
    public double drive(double distanceToEnd,
                                          double tangentialVelocity) {
        // Tuning
        double brakingDeceleration = 294;
        double coastingDeceleration = 40;
        double brakingPower = -0.2;
        double velocityP = 0.015;
        
        double quadratic = 0.0015;
        double linearLag = 0.045;
        
        double brakingEstimation = 1;
        // >1 -> overestimates braking - brakes early
        // <1 -> underestimates braking - overshoots
        
        // Constraints
        double velocityToCoastToBeforeBraking = 99;
        double maxVelocity = 99;
        double maxAcceleration = 9999;
        
        // Coast PIDF
        double kV = 0.015;
        double kS = 0.02;
        double kP = 0.01;
        double kAccel = 0.001;
        double kDecel = 0.0005;
        
        double brakingDistance =
            follower.positionalController.getPredictedBrakingDisplacement(tangentialVelocity)
                * brakingEstimation;
        // (linear term accounts for braking lag)
        
        boolean wasBraking = shouldBrake;
        shouldBrake =
            shouldBrake || brakingDistance >= distanceToEnd;
        
        double nextBrakedVelocity =
            tangentialVelocity - brakingDeceleration * follower.getDeltaTime();
        boolean switchToTranslational =
            (wasBraking && nextBrakedVelocity < 0);
        
        double driveError;
        double feedforward;
        if (wasBraking) { // brake PIDF
            double targetVelocity =
                Math.signum(distanceToEnd) *
                    Math.sqrt(Math.abs(-2 * -brakingDeceleration * brakingEstimation * distanceToEnd
                    ));
            
            driveError = (targetVelocity - tangentialVelocity);
            feedforward = brakingPower;
        }
        else if (shouldBrake) { // initialBraking PIDF (may be merged with other brake
            // PIDF)
            // Only runs once when first braking.
            double k2 = quadratic * brakingEstimation; // decel
            double k1 = linearLag; // motor lag term
            
            double discrim = k1 * k1 + 4 * k2 * distanceToEnd;
            double targetVelocity =
                (-k1 + Math.signum(discrim) * Math.sqrt(Math.abs(discrim))) / (2 * k2);
            
            driveError = (targetVelocity - tangentialVelocity);
            
            // if braking add -0.2 feedforward otherwise add kV, kS, and kA
            // or try tuning kA enough to overcome kV and decelerate
            feedforward = brakingPower;
        }
        else { // accel + coast PIDF
            double idealVelocity =
                Math.signum(distanceToEnd) *
                    Math.sqrt(Math.max(0,
                                       velocityToCoastToBeforeBraking * velocityToCoastToBeforeBraking
                                           - 2 * -coastingDeceleration * Math.abs(distanceToEnd)
                    ));
            
            double maxAccelVelocity = tangentialVelocity + maxAcceleration * follower.getDeltaTime();
            double targetVelocity = Math.min(Math.min(maxAccelVelocity, maxVelocity),
                                             idealVelocity);
            
            double accelError = targetVelocity - tangentialVelocity;
            double targetAccel = accelError / follower.getDeltaTime();
            
            // idk if we want different accel and decel coefficients in coasting regime
            double accelCoeff = (accelError > 0) ? kAccel : kDecel;
            
            driveError = (targetVelocity - tangentialVelocity);
            feedforward = kV * (targetVelocity) + targetAccel + accelCoeff + kS;
            
            return
                Range.clip(kP * driveError + feedforward, 0, 1);
        }
        
        return velocityP * driveError + feedforward;
    }
    
    
    public double drive2(double distanceRemaining,
                        double tangentialVelocity) {
        // Tuning
        double brakingDeceleration = 294;
        double brakingPower = -0.2;
        
        double quadratic = 0.0015;
        double linearLag = 0.045;
        
        double brakingEstimation = 1;
        // >1 -> overestimates braking - brakes early
        // <1 -> underestimates braking - overshoots
        
        // Constraints
        double maxVelocity = 70;
        double maxAcceleration = 9999;
        
        // PIDF
        double kP = 0.015;
        double kV = 0.015;
        double kS = 0.01;
        double kA = 0.00035; // 0.015 * 60 - 300x = -0.2
        
        double k2 = quadratic * brakingEstimation; // decel
        double k1 = linearLag; // motor lag term
        
//        double discrim = k1 * k1 + 4 * k2 * distanceRemaining;
//        double velocityToBrakeInTime =
//            (-k1 + Math.signum(discrim) * Math.sqrt(Math.abs(discrim))) / (2 * k2);
        double velocityToBrakeInTime = Math.signum(distanceRemaining) *
            (-k1 + Math.sqrt(k1 * k1 + 4 * k2 * Math.abs(distanceRemaining))) / (2 * k2);
        
//        double maxAccelVelocity = tangentialVelocity + maxAcceleration * follower.getDeltaTime();
//        double velocityConstraint = Math.min(maxAccelVelocity, maxVelocity);
        double velocityConstraint = maxVelocity;
        double targetVelocity = Math.min(velocityConstraint, velocityToBrakeInTime);
        
        double driveError = (targetVelocity - tangentialVelocity);
        
        shouldBrake =
            shouldBrake || tangentialVelocity >= velocityToBrakeInTime;
        
        double feedforward;
        double nextVelocity =
            tangentialVelocity + -brakingDeceleration * follower.getDeltaTime();
//        // velocityToBrakeInTime < 0 which is when it overshoots
//        // next velocity is negative
//        double predictedDistanceRemaining =
//            distanceRemaining - tangentialVelocity * follower.getDeltaTime();

        isHolding = isHolding || (shouldBrake && nextVelocity < 0);
        
        if (isHolding) {
            // this is predictive braking translational controller rn but
            // could be something else to hold position
            
            // reset integral if nextVelocityIfBroke > 0
            // add integral of (error - prediction)
            return follower.positionalController.computeOutput(distanceRemaining
                , tangentialVelocity);
        }
        if (shouldBrake) { // isBraking
            feedforward = brakingPower;
            
            // linear lag term is now basically gone
        }
        else {
            feedforward =
                kV * targetVelocity + kS * Math.signum(driveError);
            // if driveError is negative this force will be applied -kB *
            // tangentialVelocity
            // so to counter act it you can add it, stay positive if it makes it positive?
        }
        
        follower.telemetry.addData("targetVelocity", targetVelocity);
        follower.telemetry.addData("currentVelocity", tangentialVelocity);
        
        return kP * driveError + feedforward;
    }
    
    public double getCoastBrakePIDFPower(double distanceToEnd, double tangentialVelocity) {
        // Tuning
        double brakingDeceleration = 294;
        double coastingDeceleration = 40;
        double brakingPower = -0.2;
        
        // Constraints
        double velocityToCoastToBeforeBraking = 99;
        double maxVelocity = 99;
        double maxAcceleration = 9999;
        
        double brakingDistance =
            follower.positionalController.getPredictedBrakingDisplacement(tangentialVelocity);
        
        // linear term accounts for braking lag
        
        boolean wasBraking = shouldBrake;
        shouldBrake =
            shouldBrake || brakingDistance >= distanceToEnd;
        
        double nextVelocity =
            tangentialVelocity + -brakingDeceleration * follower.getDeltaTime();
        isHolding = isHolding || (wasBraking && nextVelocity < 0);

        double tangentPower;
        if (isHolding) {
            // this is predictive braking translational controller rn but
            // could be something else to hold position
            
            // reset integral if nextVelocityIfBroke > 0
            // add integral of (error - prediction)
            tangentPower = follower.positionalController.computeOutput(distanceToEnd
                , tangentialVelocity);
            
            // shouldBrake = brakingDistance >= distanceToEnd && nextVelocityIfBroke > 0
        }
        else if (wasBraking) { // brake PIDF
            double idealVelocity =
                Math.signum(distanceToEnd) *
                    Math.sqrt(Math.abs(-2 * -brakingDeceleration * distanceToEnd
                    ));

            follower.telemetry.addData("targetVel", idealVelocity);
            follower.telemetry.addData("nextVelocity", nextVelocity);
            follower.telemetry.addData("brakingDistance", brakingDistance);

            tangentPower =
                brakingPower + 0.015 * (idealVelocity - tangentialVelocity);
        }
        else if (shouldBrake) {
            // initialBraking - only runs one first loop of braking
            // torque ramp-up + backEMF -> wheels stopping
            // need to account for linear + quadratic in this loop
            
            double k2 = 0.0015;
            double k1 = 0.045;

            double discrim = k1 * k1 + 4 * k2 * distanceToEnd;
            double targetVelocity =
                (-k1 + Math.signum(discrim) * Math.sqrt(Math.abs(discrim))) / (2 * k2);
            
            tangentPower = brakingPower + 0.015 * (targetVelocity - tangentialVelocity);
            //                + follower.positionalController.computeOutput(distanceToEnd
            //                , tangentialVelocity);
        }
        else { // accel + coast PIDF
            double idealVelocity =
                Math.signum(distanceToEnd) *
                    Math.sqrt(Math.max(0,
                                       velocityToCoastToBeforeBraking * velocityToCoastToBeforeBraking
                                           - 2 * -coastingDeceleration * Math.abs(distanceToEnd)
                    ));
            
            double maxAccelVelocity = tangentialVelocity + maxAcceleration * follower.getDeltaTime();
            double targetVelocity = Math.min(Math.min(maxAccelVelocity, maxVelocity),
                                             idealVelocity);
            
            double accelError = targetVelocity - tangentialVelocity;
            double targetAccel = accelError / follower.getDeltaTime();
            
            double accelCoeff = (accelError > 0) ? 0.001 : 0.0005;
            
            // (targetVelocity - tangentialVelocity); / deltaTime
            // endVelocity
            
            tangentPower =
                Range.clip(0.01 * (targetVelocity - tangentialVelocity) + 0.015 * (targetVelocity + 0.00 * targetAccel + 0.02)
                    , 0,
                           1);
            
            
            follower.telemetry.addData("targetVel", targetVelocity);
            follower.telemetry.addData("nextVelocity", nextVelocity);
            follower.telemetry.addData("brakingDistance", brakingDistance);
        }
        return tangentPower;
    }

    public double positionalControl(double distanceToEnd, double tangentialVelocity) {
        // Tuning
        double brakingDeceleration = 294;
        double brakingPower = -0.2;
        
        double brakingDistance =
            follower.positionalController.getPredictedBrakingDisplacement(tangentialVelocity);
        
        boolean wasBraking = shouldBrake;
        shouldBrake =
            shouldBrake || brakingDistance >= distanceToEnd;

        double nextVelocity =
            tangentialVelocity - brakingDeceleration * follower.getDeltaTime();
        isHolding = isHolding || (wasBraking && nextVelocity < 0);

        double tangentPower;
        if (isHolding) {
            tangentPower = follower.positionalController.computeOutput(distanceToEnd
                , tangentialVelocity);
        }
        else if (wasBraking) {
            double predictedStop = (tangentialVelocity * tangentialVelocity) / (2 * brakingDeceleration);
            tangentPower = brakingPower +
                0.2 * (distanceToEnd - predictedStop);
        }
        else if (shouldBrake) {
            tangentPower = brakingPower
                + follower.positionalController.computeOutput(distanceToEnd
                , tangentialVelocity);
        }
        else { // cruise
            return 1;
        }

        return tangentPower;
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
        return shouldBrake;
    }
    
    public String getName() {
        return "Path{" + endPose.toString() + ",lastTValue=" + lastTValue + "," +
            "isFinished=" + isFinished() + "}";
    }
}
