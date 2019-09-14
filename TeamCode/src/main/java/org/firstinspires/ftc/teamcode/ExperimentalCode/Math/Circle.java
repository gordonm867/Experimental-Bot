package org.firstinspires.ftc.teamcode.ExperimentalCode.Math;

import java.io.Serializable;

// SOURCE: https://github.com/Lanchon/circle-circle-intersection

public final class Circle implements Serializable {

    private static final long serialVersionUID = 1L;

    public final Vector2 c;
    public final double r;
    private Point center;

    public Circle(Vector2 c, double r) {
        if (!(r > 0)) throw new IllegalArgumentException("Radius must be positive");
        this.c = c;
        this.r = r;
        if(c != null) {
            center = new Point(c.x, c.y);
        }
        else {
            center = new Point(0, 0);
        }
    }

    public Circle(Point center, double radius) {
        this.center = center;
        this.r = radius;
        this.c = new Vector2(center.getX(), center.getY());
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((c == null) ? 0 : c.hashCode());
        long temp;
        temp = Double.doubleToLongBits(r);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        Circle other = (Circle)obj;
        if (c == null) {
            if (other.c != null)
                return false;
        } else if (!c.equals(other.c))
            return false;
        return Double.doubleToLongBits(r) == Double.doubleToLongBits(other.r);
    }

    public Point getCenter() {
        return center;
    }

    public double getRadius() {
        return r;
    }

    public double getArea() {
        return Math.pow(r, 2) * Math.PI;
    }

    public double getCircumference() {
        return 2 * Math.PI * r;
    }

    @Override
    public String toString() {
        return getClass().getSimpleName() + "(c: " + c + ", r: " + r + ")";
    }

}