package org.firstinspires.ftc.teamcode.blackice.core.geometry;

import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

/**
 * A immutable, stateless part of a path, purely positional. It only knows it's tangent heading.
 * The shape of paths.
 */
public interface PathGeometry {
    PathPoint computeClosestPathPointTo(Vector robotPosition, double startingGuess);

    double length();

    PathPoint getEndPathPoint();
    
    default Vector getEndPoint() {
        return getEndPathPoint().point;
    }

    PathGeometry reversed();
    PathGeometry mirrored();
    
    default double getDistanceRemaining(PathPoint closestPoint) {
        double distanceToEnd;
        if (closestPoint.distanceRemaining == 0) {
            distanceToEnd =
                getEndPoint()
                    .minus(closestPoint.point)
                    .dot(Vector.fromPolar(1, closestPoint.tangent));
        } else {
            distanceToEnd = closestPoint.distanceRemaining;
        }
        return distanceToEnd;
    }
}
