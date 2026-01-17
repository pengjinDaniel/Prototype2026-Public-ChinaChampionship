package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PolarVector represents the relative distance and direction of a pair of objects in 2D space.
 */
public class PolarVector {
    protected final double magnitude;
    protected final DistanceUnit distanceUnit;
    protected final DistanceUnit defaultDistanceUnit = DistanceUnit.INCH;
    protected final double direction;
    protected final AngleUnit headingUnit;
    protected final AngleUnit defaultHeadingUnit = AngleUnit.RADIANS;

    /**
     * Creates a new PolarVector object.
     * @param distanceUnit the unit of distance for magnitude
     * @param magnitude the distance
     * @param headingUnit the unit of heading
     * @param direction the direction
     */
    public PolarVector(DistanceUnit distanceUnit,double magnitude, AngleUnit headingUnit, double direction) {
        this.magnitude = magnitude;
        this.distanceUnit = distanceUnit;
        this.direction = direction;
        this.headingUnit = headingUnit;
    }

    /**
     * Creates a new PolarVector object.
     * @param distanceUnit the unit of distance for magnitude
     * @param x the x coordinate of the polar vector
     * @param y the y coordinate of the polar vector
     */
    public PolarVector(DistanceUnit distanceUnit,double x, double y) {
        this.magnitude = Math.sqrt(x * x + y * y);
        this.distanceUnit = distanceUnit;
        this.direction = Math.atan2(y, x);
        this.headingUnit = defaultHeadingUnit;
    }

    /**
     * Creates a new PolarVector object in the default units.
     * @param magnitude the distance
     * @param direction the direction
     */
    public PolarVector(double magnitude, double direction) {
        this.magnitude = magnitude;
        this.distanceUnit = defaultDistanceUnit;
        this.direction = direction;
        this.headingUnit = defaultHeadingUnit;
    }

    /**
     * This gets Magnitude in the desired distance unit
     * @param unit the desired distance unit
     * @return the Magnitude converted to the desired distance unit
     */
    public double getMagnitude(DistanceUnit unit) {
        return unit.fromUnit(this.distanceUnit, magnitude);
    }

    /**
     * This gets Magnitude in the default distance unit
     * @return the Magnitude converted to the default distance unit
     */
    public double getMagnitude() {
        return magnitude;
    }

    /**
     * This gets the Direction in the desired angle unit
     * Be aware that this normalizes the angle to be between -PI and PI for RADIANS or
     * between -180 and 180 for DEGREES
     * @param unit the desired angle unit
     * @return the Direction converted to the desired angle unit
     */
    public double getHeading(AngleUnit unit) {
        return unit.fromUnit(this.headingUnit, direction);
    }

    /**
     * This gets Direction in the default distance unit
     * @return the Direction converted to the default distance unit
     */
    public double getHeading() {
        return direction;
    }

    /**
     * This returns a string representation of the object in a human readable format for debugging purposes.
     * @return a string representation of the object
     */
    @NonNull
    @Override
    public String toString() {
        return "(PolarVector) magnitude=" + magnitude + " " + distanceUnit + ", direction=" + direction + " " + headingUnit;
    }
}
