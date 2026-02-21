package org.firstinspires.ftc.teamcode.blackice.geometry;

public class PoseConstructor {
    Vector robotSize;
    
    public PoseConstructor(Vector robotSize) {
        this.robotSize = robotSize;
    }
    
    private static double robotLength;
    private static double robotWidth;
    
    public static void configureRobot(double length, double width) {
        robotLength = length;
        robotWidth = width;
    }
    
    public Pose rotatedOffset(double forward, double left) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        
        double fieldX = forward * cos - left * sin;
        double fieldY = forward * sin + left * cos;
        
        return new Pose(x + fieldX, y + fieldY, heading);
    }
    
    public Pose alignCenterTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading);
    }
    
    public Pose alignFrontTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading)
            .rotatedOffset(-robotLength / 2.0, 0);
    }
    
    public Pose alignBackTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading)
            .rotatedOffset(robotLength / 2.0, 0);
    }
    
    public Pose alignLeftSideTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading)
            .rotatedOffset(0, robotWidth / 2.0);
    }
    
    public Pose alignRightSideTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading)
            .rotatedOffset(0, -robotWidth / 2.0);
    }
    
    public Pose alignFrontLeftCornerTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading)
            .rotatedOffset(-robotLength / 2.0, robotWidth / 2.0);
    }
    
    public Pose alignFrontRightCornerTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading)
            .rotatedOffset(-robotLength / 2.0, -robotWidth / 2.0);
    }
    
    public Pose alignBackLeftCornerTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading)
            .rotatedOffset(robotLength / 2.0, robotWidth / 2.0);
    }
    
    public Pose alignBackRightCornerTo(double targetX, double targetY) {
        return new Pose(targetX, targetY, heading)
            .rotatedOffset(robotLength / 2.0, -robotWidth / 2.0);
    }
    
}
