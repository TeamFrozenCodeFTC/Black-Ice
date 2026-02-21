package org.firstinspires.ftc.teamcode.blackice.core.geometry;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

public class WaypointGeometry implements PathGeometry {
    private final double length;
    private final Vector startPoint;
    private final Vector tangent;
    private final PathPoint endPathPoint;
    
    public WaypointGeometry(Vector start, Vector end) {
        this.startPoint = start;
        Vector displacement = end.minus(start);
        this.length = displacement.computeMagnitude();
        this.tangent = displacement.dividedBy(length);
        this.endPathPoint = new PathPoint(end, tangent.getAngle(), 0, length, 1, 1);
    }

    @Override
    public double length() {
        return length;
    }
    
    @Override
    public PathPoint getEndPathPoint() {
        return endPathPoint;
    }
    
    @Override
    public PathPoint computeClosestPathPointTo(Vector point, double startingGuess) {
        Vector startToPoint = point.minus(startPoint);
        double t = Range.clip(startToPoint.dot(tangent) / length, 0, 1);
        double distanceAlongPath = t * length;
        return new PathPoint(endPathPoint.point,
                             point.minus(endPathPoint.point).getAngle(), 0,
                             length - distanceAlongPath, t, t);
    }
    
    @Override
    public LineGeometry reversed() {
        return new LineGeometry(endPathPoint.point, startPoint);
    }
    
    @Override
    public LineGeometry mirrored() {
        return new LineGeometry(startPoint.mirroredAcrossYAxis(), endPathPoint.point.mirroredAcrossYAxis());
    }
}
