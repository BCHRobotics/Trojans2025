package frc.utils;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/*
 * This script is for helper functions related to vision and related math
 * There should only be public static functions in here
 */
public class MathUtils {
    /**
     * A function that, given a transform2d as a vector, rotates it by an angle
     * @param inputMatrix The input vector
     * @param angle Angle to rotate by IN RADIANS
     * @return the vector rotated by the angle
     */
    public static Translation2d applyRotationMatrix(Translation2d inputMatrix, double angle) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(angle);
        double cosHeading = Math.cos(angle);

        // Create field-relative coordinates using the heading and robot-relative coords
        double fieldX = inputMatrix.getX() * cosHeading + inputMatrix.getY() * -sinHeading;
        double fieldY = inputMatrix.getX() * sinHeading + inputMatrix.getY() * cosHeading;

        // Create the transform2d object
        Translation2d rotatedVector = new Translation2d(fieldX, fieldY);

        return rotatedVector;
    }

    public static double getDistance(Pose2d a, Pose2d b) {
        double difX = b.getX() - a.getX();
        double difY = b.getY() - a.getY();

        return Math.sqrt(difX * difX + difY * difY);
    }

    public static boolean lineIntersect(Vector4 a, Vector4 b) {
        Vector2 s1 = new Vector2(a.x, a.y);
        Vector2 d1 = Vector2.normalize(new Vector2(a.z, a.w));
        Vector2 s2 = new Vector2(b.x, b.y);
        Vector2 d2 = Vector2.normalize(new Vector2(b.z, b.w));

        double xDiff = s2.x - s1.x;
        double yDiff = s2.y - s1.y;
        double det = d2.x * d1.y - d2.y * d1.x;
        double u = (yDiff * d2.x - xDiff * d2.y) / det;
        double v = (yDiff * d1.x - xDiff * d1.y) / det;

        return v>0&&u>0&&v<=Vector2.magnitude(new Vector2(b.z, b.w))&&u<=Vector2.magnitude(new Vector2(a.z, a.w));
    }

    public static double getIntersectDist(Vector4 a, Vector4 b) {
        Vector2 s1 = new Vector2(a.x, a.y);
        Vector2 d1 = Vector2.normalize(new Vector2(a.z, a.w));
        Vector2 s2 = new Vector2(b.x, b.y);
        Vector2 d2 = Vector2.normalize(new Vector2(b.z, b.w));

        double xDiff = s2.x - s1.x;
        double yDiff = s2.y - s1.y;
        double det = d2.x * d1.y - d2.y * d1.x;
        double u = (yDiff * d2.x - xDiff * d2.y) / det;
        double v = (yDiff * d1.x - xDiff * d1.y) / det;

        if (v>0&&u>0&&v<=Vector2.magnitude(new Vector2(b.z, b.w))&&u<=Vector2.magnitude(new Vector2(a.z, a.w))) {
            return u;
        } else {return -1; }
    }

    // find the midpoint of the polygon that the segment corresponds to
    public static Vector2 getMidpoint(Vector4 segment, Polygon[] polygons) {
        for (int i = 0; i < polygons.length; i++) {
            for (int j = 0; j < polygons[i].points.length; j++) {
                Vector4 lineSegment;
                if (j < polygons[i].points.length - 1) {
                    lineSegment = makeLineFromVector2(polygons[i].points[j], polygons[i].points[j+1]);
                } else {lineSegment = makeLineFromVector2(polygons[i].points[j], polygons[i].points[0]);}

                if (lineSegment == segment) {
                    return polygons[i].GetMidpoint();
                }
            }
        }

        return new Vector2(-1, -1);
    }

    // travelling from one point to another, where is the hit point?
    public static Vector2 getInterruptPoint(Vector2 a, Vector2 b, Polygon[] polygons) {
        List<Vector2> hits = new LinkedList<Vector2>();

        for (int i = 0; i < polygons.length; i++) {
            for (int j = 0; j < polygons[i].points.length; j++) {
                Vector4 lineSegment;
                if (j < polygons[i].points.length - 1) {
                    lineSegment = makeLineFromVector2(polygons[i].points[j], polygons[i].points[j+1]);
                } else {lineSegment = makeLineFromVector2(polygons[i].points[j], polygons[i].points[0]);}

                double checkDist = getIntersectDist(makeLineFromVector2(a, b), lineSegment);
                if (checkDist != -1) {
                    Vector2 dir = Vector2.normalize(new Vector2(b.x-a.x, b.y-a.y));
                    hits.add(Vector2.add(a,new Vector2(dir.x * checkDist, dir.y * checkDist)));
                }
            }
        }

        if (hits.size() == 0) {
            return new Vector2(-1, -1);
        }

        Vector2 king = hits.get(0);
        if (hits.size() > 1) {
            for (int i = 1; i < hits.size(); i++) {
                if (Vector2.distance(hits.get(i), a) < Vector2.distance(king, a)) {
                    king = hits.get(i);
                }
            }
        }

        return king;
    }

    // travelling from one point to another, where is the hit point?
    public static Vector4 getInterruptSegment(Vector2 a, Vector2 b, Polygon[] polygons) {
        List<Vector2> hits = new LinkedList<Vector2>();
        List<Vector4> segments = new LinkedList<Vector4>();

        for (int i = 0; i < polygons.length; i++) {
            for (int j = 0; j < polygons[i].points.length; j++) {
                Vector4 lineSegment;
                if (j < polygons[i].points.length - 1) {
                    lineSegment = makeLineFromVector2(polygons[i].points[j], polygons[i].points[j+1]);
                } else {lineSegment = makeLineFromVector2(polygons[i].points[j], polygons[i].points[0]);}

                double checkDist = getIntersectDist(makeLineFromVector2(a, b), lineSegment);
                if (checkDist != -1) {
                    Vector2 dir = Vector2.normalize(new Vector2(b.x-a.x, b.y-a.y));
                    hits.add(Vector2.add(a,new Vector2(dir.x * checkDist, dir.y * checkDist)));
                    segments.add(lineSegment);
                }
            }
        }

        if (hits.size() == 0) {
            return new Vector4(-1, -1, -1, -1);
        }

        int kingIndex = 0;
        if (hits.size() > 1) {
            for (int i = 1; i < hits.size(); i++) {
                if (Vector2.distance(hits.get(i), a) < Vector2.distance(hits.get(kingIndex), a)) {
                    kingIndex = i;
                }
            }
        }

        return segments.get(kingIndex);
    }

    public static Vector4 makeLineFromVector2(Vector2 a, Vector2 b) {
        return new Vector4(a.x, a.y, b.x - a.x, b.y - a.y);
    }
    public static Vector4 makeLineFromPoses(Pose2d a, Pose2d b) {
        return new Vector4(a.getX(), a.getY(), b.getX() - a.getX(), b.getY() - a.getY());
    }

    // this code is independent, only used to color the debug path and stop the iteration while loop
    public static boolean isPathValid(Pose2d[] pathPoints, Polygon[] polygons) {
        for (int i = 0; i < polygons.length; i++) {
            for (int j = 0; j < polygons[i].points.length; j++) {
                for (int k = 0; k < pathPoints.length - 1; k++) {
                    Vector4 lineSegment;
                    if (j < polygons[i].points.length - 1) {
                        lineSegment = makeLineFromVector2(polygons[i].points[j], polygons[i].points[j+1]);
                    } else {lineSegment = makeLineFromVector2(polygons[i].points[j], polygons[i].points[0]);}

                    if (j == 0) {
                        //System.out.println(polygons[i].points[j].x + "   " + polygons[i].points[j].y);
                        //System.out.println(lineSegment.x + "  " + lineSegment.y + "  " + lineSegment.z + "  " + lineSegment.w);
                    }

                    if (lineIntersect(makeLineFromPoses(pathPoints[k], pathPoints[k+1]), lineSegment)) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    public static Pose2d[] pose2dArray(List<Pose2d> poseList) {
        Pose2d[] toReturn = new Pose2d[poseList.size()];

        for (int i = 0; i < toReturn.length; i++) {
            toReturn[i] = poseList.get(i);
        }

        return toReturn;
    }

    public static Pose2d[] calculatePath(Pose2d start, Pose2d end, Polygon[] polygons) {
        List<Pose2d> pathPoints = new LinkedList<Pose2d>();
        pathPoints.add(start);
        pathPoints.add(end);

        int safeIterations = 10;
        int iterationCount = 0;

        Vector2 midpoint = new Vector2(-1, -1);

        // loop ends if the path is valid or we run out of time
        while(iterationCount < safeIterations && !isPathValid(pose2dArray(pathPoints), polygons)) {
            Vector2 hitPoint = new Vector2();
            Vector4 hitSegment = new Vector4();
            int hitIndex = -1;

            for (int i = 0; i < pathPoints.size() - 1; i++) {
                // get the interrupting point
                Vector2 interrupt;
                
                // write a function to make sure the equals works
                if (!Vector2.equals(midpoint, new Vector2(-1, -1))) {
                    interrupt = getInterruptPoint(new Vector2(pathPoints.get(i).getX(), pathPoints.get(i).getY()), new Vector2(midpoint.x, midpoint.y), polygons);
                } else {
                    interrupt = getInterruptPoint(new Vector2(pathPoints.get(i).getX(), pathPoints.get(i).getY()), new Vector2(pathPoints.get(i+1).getX(), pathPoints.get(i+1).getY()), polygons);
                }
                
                if (!Vector2.equals(interrupt, new Vector2(-1, -1))) {
                    hitPoint = Vector2.lerp(new Vector2(pathPoints.get(i).getX(), pathPoints.get(i).getY()), interrupt, 0.9f);
                    hitSegment = getInterruptSegment(new Vector2(pathPoints.get(i).getX(), pathPoints.get(i).getY()), new Vector2(pathPoints.get(i+1).getX(), pathPoints.get(i+1).getY()), polygons);
                    hitIndex=i;
                }
            }

            if (hitIndex == -1) {System.out.println("there was a problem!!"); return null; }

            int count = 0;
            while(count < 10 && !Vector2.equals(getInterruptPoint(hitPoint, Vector2.lerp(hitPoint, new Vector2(hitSegment.x, hitSegment.y), 0.9f), polygons),new Vector2(-1, -1))) {
                hitSegment = getInterruptSegment(hitPoint, new Vector2(hitSegment.x, hitSegment.y), polygons);
                count++;
            }
            
            Vector2 poseVector = Vector2.add(new Vector2(hitSegment.x, hitSegment.y),new Vector2(-hitSegment.z * 0.1f, -hitSegment.w * 0.1f));
            pathPoints.add(hitIndex + 1, new Pose2d(poseVector.x, poseVector.y, new Rotation2d()));

            iterationCount++;

            midpoint = getMidpoint(hitSegment, polygons);
        }

        return pose2dArray(pathPoints);
    }
}