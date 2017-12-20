package com.github.ultrax3.purepursuit;

import java.util.*;

public class Vehicle {

    private final double lateralWheelDistance;
    private final double v_rMax,v_lMax,v_lMin, v_rMin;

    private double v_l,v_r,curvature, rotVelocity,r,estTime,tangentialSpeed, arclength, distanceRotated;


    public String getMarkovState() {
        return "Vehicle{" +
                "v_l: " + v_l +
                ", v_r: " + v_r +
                ", curvature: " + curvature +
                ", rotVelocity: " + rotVelocity +
                ", r: " + r +
                ", estTime: " + estTime +
                ", tangentialSpeed: " + tangentialSpeed +
                ", arclength: " + arclength +
                ", distanceRotated: " + distanceRotated +
                '}';
    }

    private Vector location;
    private float angle;
    private Vector nextWaypoint;
    private Deque<Vector> waypoints = new ArrayDeque<>();

    /**
     *
     * @param location
     * @param angle
     * @param latWheelDistance the lateral distance between wheels
     * @param v_lMax the max tangential velocity the left wheels can go
     * @param v_rMax the max tangential velocity the right wheels can go
     */
    public Vehicle(Vector location, float angle, double latWheelDistance, double v_lMin, double v_lMax, double v_rMin, double v_rMax) {
        this.location = location;
        this.angle = angle;
        this.lateralWheelDistance = latWheelDistance;
        this.v_lMax = v_lMax;
        this.v_rMax = v_rMax;
        this.v_lMin = v_lMin;
        this.v_rMin = v_rMin;
    }

    /**
     *
     * @return A set of waypoints with absolute x,y,yaw (heading), curvature of path @ point, distance from beginning of path
     */
    public Deque<Vector> getWaypoints() {
        return waypoints;
    }

    /**
     * @return calculates the curvature to a waypoint.
     * <ol>
     *     <li>curvature is rate of change of direction with respect to how far you are on the line.
     *     When turning clockwise, curvature is negative. When turning counterclockwise, curvature is positive.
     *     </li>
     * </ol>
     */
    double curvatureToWaypoint(){
        double lSquared = nextWaypoint.getMagnitudeSquared(); // x^2 + y^2 = l^2 (length)
        return -2*nextWaypoint.get(0)/lSquared;
    }

    Vector getWheelTangentialVelocity(){
        curvature = curvatureToWaypoint();
        double c = 2/(lateralWheelDistance*curvature); //TODO: !!! this should be 2/(lateralWheelDistance*curvature), but this doesn't return consistent answers
        double velLeftToRightRatio = -( c + 1 ) / (1-c);
        double velRightToLeftRatio = 1/velLeftToRightRatio;
        double score = -Integer.MAX_VALUE;
        Vector bestVector = null;
        //TODO: fix clumsy way of optimizing :(

        double v_r = v_lMax * velLeftToRightRatio;
        if(MathUtils.Algebra.between(v_rMin, v_r,v_rMax)){
            score = Math.abs(v_lMax+v_r);
            bestVector = new Vector(v_lMax,v_r);
        }

        v_r = v_lMin * velLeftToRightRatio;
        if(MathUtils.Algebra.between(v_rMin, v_r,v_rMax)){
            double tempScore = Math.abs(v_lMin + v_r);
            if(tempScore > score){
                score = tempScore;
                bestVector = new Vector(v_lMin,v_r);
            }
        }

        double v_l = v_rMax * velRightToLeftRatio;
        if(MathUtils.Algebra.between(v_lMin, v_l,v_lMax)){
            double tempScore = Math.abs(v_lMax + v_l);
            if(tempScore > score){
                score = tempScore;
                bestVector = new Vector(v_l,v_rMax);
            }
        }

        v_l = v_rMin * velRightToLeftRatio;
        if(MathUtils.Algebra.between(v_lMin, v_l,v_lMax)){
            double tempScore = Math.abs(v_lMin + v_l);
            if(tempScore > score){
                bestVector = new Vector(v_l,v_rMin);
            }
        }
        this.v_l = bestVector.get(0);
        this.v_r = bestVector.get(1);
        rotVelocity = (bestVector.get(1) - bestVector.get(0))/lateralWheelDistance;

        // Note this can be negative
        r = 1/curvature; // or lateralWheelDistance/2 * (bestVector.get(1) + bestVector.get(0))/( bestVector.get(1)-bestVector.get(0));

        distanceRotated = MathUtils.Arithmetic.sign(rotVelocity)*Math.atan(nextWaypoint.get(1) / (Math.abs(r)-nextWaypoint.get(0)));

        tangentialSpeed = Math.abs(r*rotVelocity);
        arclength = distanceRotated *r;
        estTime = distanceRotated/rotVelocity;
        return bestVector;
    }

    public double getV_rMax() {
        return v_rMax;
    }

    public double getV_lMax() {
        return v_lMax;
    }

    public double getV_lMin() {
        return v_lMin;
    }

    public double getV_rMin() {
        return v_rMin;
    }

    public double getV_l() {
        return v_l;
    }

    public double getV_r() {
        return v_r;
    }

    public boolean grabNextWaypoint() {
        Vector poll = waypoints.poll();
        nextWaypoint = absoluteToRelativeCoord(poll);
        return true;
    }

    /**
     *
     * @param waypoint The "absolute" coordinate of the waypoint when facing N (assuming angle is 0 when N)
     * @return the relative coordinate where the y axis goes along the current direction of the robot
     */
    Vector absoluteToRelativeCoord(Vector waypoint){
        if(waypoint.dimensions() != 2)
            throw new IllegalArgumentException("Must be in R2");
        Vector coordDif = waypoint.clone().subtractBy(location);
        return MathUtils.LinearAlgebra.rotate2D(coordDif,angle);
    }
}
