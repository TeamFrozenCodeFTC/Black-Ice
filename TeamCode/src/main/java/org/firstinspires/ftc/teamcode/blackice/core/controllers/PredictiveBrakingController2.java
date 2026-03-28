package org.firstinspires.ftc.teamcode.blackice.core.controllers;

@Deprecated
public class PredictiveBrakingController2 {
    public double kBraking;
    public double kFriction;
    public double kNonInitial;
    public double kP;

    public double maximumBrakingPower = 0.2;
    public boolean isBraking = false;
    public double holdingVelocityThreshold = 99;
    public double velocity;
    
    public PredictiveBrakingController2(double kP, double kLinearBraking,
                                        double kQuadraticFriction, double kNonInitial) {
        this.kP = kP;
        this.kBraking = kLinearBraking;
        this.kFriction = kQuadraticFriction;
        this.kNonInitial = kNonInitial;
    }
    
    public double computeOutput(double error,
                                double predictedBrakingDisplacement) {
        return kP * (error - predictedBrakingDisplacement)
            + (Math.abs(velocity) > holdingVelocityThreshold ? maximumBrakingPower : 0);
    }
    
    public double getInitialBrakingDisplacement() {
        return Math.abs(velocity) * velocity * kFriction + velocity * kBraking;
    }
    
    public double getBrakingDisplacement() {
        return Math.abs(velocity) * velocity * kFriction + velocity * kBraking;
        //return Math.abs(velocity) * velocity * kNonInitial;
    }
    
    public double update(double distanceToTarget, double velocityTowardTarget) {
        velocity = velocityTowardTarget;
        double initialBrakingDistance = getInitialBrakingDisplacement();
        
        boolean wasBraking = isBraking;
        isBraking = initialBrakingDistance >= distanceToTarget;
        
        if (isBraking & !wasBraking) {
            return computeOutput(distanceToTarget, initialBrakingDistance);
        } else if (isBraking & wasBraking) {
            return computeOutput(distanceToTarget, getBrakingDisplacement());
        } else {
            return kP * (distanceToTarget - initialBrakingDistance); // full power
        }
    }
//
//    public double update2(double distanceToTarget, double velocityTowardTarget) {
//        velocity = velocityTowardTarget;
//        double initialBrakingDistance = getInitialBrakingDisplacement();
//
//        boolean wasBraking = isBraking;
//        isBraking = initialBrakingDistance >= distanceToTarget;
//
//        if (isBraking & !wasBraking) {
//            return computeOutput(distanceToTarget, initialBrakingDistance);
//        } else if (isBraking & wasBraking) {
//            return computeOutput(distanceToTarget, getBrakingDisplacement());
//        } else {
//            return 1; // full power
//        }
//    }
}
