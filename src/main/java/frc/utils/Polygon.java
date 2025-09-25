package frc.utils;

public class Polygon {
    public Vector2[] points;

    public Polygon() {}

    public Polygon(Vector2[] points) {
        this.points = points;
    }
    
    public Vector2 GetMidpoint() {
        Vector2 midpoint = new Vector2(0, 0);
        for (int i = 0; i < points.length; i++) {
            midpoint = Vector2.add(midpoint, points[i]);
        }
        midpoint = new Vector2(midpoint.x / points.length, midpoint.y / points.length);
        return midpoint;
    }
}
