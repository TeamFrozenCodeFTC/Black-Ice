//package org.firstinspires.ftc.teamcode.blackice.core;
//
//public class PredictiveBrakingModel {
//    private PredictiveBrakingCoefficients coefficients;
//
//    public PredictiveBrakingModel(PredictiveBrakingCoefficients coefficients) {
//        this.coefficients = coefficients;
//    }
//
//    public void setCoefficients(PredictiveBrakingCoefficients coefficients) {
//        this.coefficients = coefficients;
//    }
//
//    /**
//     * Note: Distance is signed.
//     */
//    public double getBrakingDistance(double velocity) {
//        return Math.abs(velocity) * velocity * coefficients.kQuadraticFriction
//            + velocity * coefficients.kLinearBraking;
//    }
//}
