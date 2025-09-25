package frc.utils;

public class Vector2 {
    public double x;
    public double y;

    public Vector2() {}

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    public static Vector2 normalize(Vector2 a) {
        double length = Math.sqrt(a.x * a.x + a.y * a.y);
        return new Vector2(a.x / length, a.y / length);
    }

    public static double magnitude(Vector2 a) {
        return Math.sqrt(a.x * a.x + a.y * a.y);
    }

    public static double distance(Vector2 a, Vector2 b) {
        return Math.sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
    }

    // c is [0..1]
    public static Vector2 lerp(Vector2 a, Vector2 b, double c) {
        return new Vector2(a.x + (b.x-a.x) * c, a.y + (b.y-a.y)*c);
    }

    public static boolean equals(Vector2 a, Vector2 b) {
        if (a.x == b.x && a.y == b.y) {
            return true;
        } else {return false;}
    }
}
