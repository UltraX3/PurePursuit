package com.github.ultrax3.purepursuit;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitMovementStrategy implements TankMovementStrategy{

    private final List<Vector> pathPoints;
    private Vector goalPoint;
    private final TankRobot tankRobot;
    private double r;
    private double rotVelocity;
    private double lookAheadDistance;
    private boolean finishedPath = false;
    double currentAngle;

    int lastPath = 0;

    private Function<Double,Vector> esimatePositionFromRotation;
    private double estimatedTime;
    private Function<Double, Vector> timeToPosition;
    private Function<Double,Double> estimateHeadingFromTime;
    private double rotatedAngle;
    private Vector usedEstimatedLocation;
    private Vector goalPointAbsolute;

    public PurePursuitMovementStrategy(TankRobot tankRobot, List<Vector> goals, double lookAheadDistance){
        this.pathPoints = goals;
        this.tankRobot = tankRobot;
        this.lookAheadDistance = lookAheadDistance;
    }

    public Function<Double, Double> getEstimateHeadingFromTime() {
        return estimateHeadingFromTime;
    }

    public Function<Double, Vector> estimateTimeToPosition() {
        return timeToPosition;
    }

    public Function<Double, Vector> getEsimatePositionFromRotation() {
        return esimatePositionFromRotation;
    }

    int counter = 0;

    List<Vector> intersections;
    public void update() {
        if(finishedPath)
            return;

        intersections = new ArrayList<>();
        int nextPathI = Integer.MAX_VALUE;
        usedEstimatedLocation = tankRobot.getEsimatedLocation();
        floop:
        for(int i =lastPath; i<= lastPath+1; i++){
            if(i+1 >= pathPoints.size())
                continue floop;
            counter++;
            Vector lineP1 = pathPoints.get(i);
            Vector lineP2 = pathPoints.get(i + 1);
            double toLookAhead = lookAheadDistance;
            List<Vector> vectorList = new ArrayList<>(MathUtils.Geometry.
                    getCircleLineIntersectionPoint(lineP1, lineP2, usedEstimatedLocation, toLookAhead));
            vectorList.removeIf(vector -> !vector.between(lineP1,lineP2));
            if(i == lastPath+1 && !vectorList.isEmpty())
                nextPathI = intersections.size();
            intersections.addAll(vectorList);
        }

        Vector toCompare = goalPointAbsolute;
        if(toCompare == null){
            toCompare = pathPoints.get(1);
        }
        int closestVectorI = closest(toCompare,intersections);
        if(closestVectorI == -1){
            finishedPath = true;
            return;
        }
        Vector closest = intersections.get(closestVectorI);
        if(closestVectorI >= nextPathI){
            lastPath++;
        }
        goalPoint = absoluteToRelativeCoord(closest);
        goalPointAbsolute = closest;

        Vector wheelTangentialVelocity = getWheelTangentialVelocity();
    }

    public double getLookAheadDistance() {
        return lookAheadDistance;
    }

    int closest(Vector origin, List<Vector> vectors){
        double minMagSquared = Double.MAX_VALUE;
        int minVectorI = -1;
        for(int i = 0; i < vectors.size(); i++){
            Vector vector = vectors.get(i);
            double magnitudeSquared = origin.subtractBy(vector).getMagnitudeSquared();
            if(magnitudeSquared < minMagSquared){
                minMagSquared = magnitudeSquared;
                minVectorI = i;
            }
        }
        return minVectorI;
    }

    /**
     * To see if goal points are continuous
     * @param 
     * @return
     */
    boolean isValidGoalPoint(Vector goalPoint){
        if(this.goalPoint == null)
            return true;
        if(this.goalPoint.get(1) < 0)
            return false;
        return true;
    }

    private double curvatureToGoal(){
        double lSquared = goalPoint.getMagnitudeSquared(); // x^2 + y^2 = l^2 (length)
        return -2*goalPoint.get(0)/lSquared;
    }

    public Vector getCircleCenter(){
        Vector circleRelativeCenter = new Vector(r,0);
        Vector circleRelativeCenterRotated = MathUtils.LinearAlgebra.rotate2D(circleRelativeCenter, currentAngle);
        return usedEstimatedLocation.add(circleRelativeCenterRotated);
    }

    public double getR() {
        return r;
    }

    private Vector getWheelTangentialVelocity(){
        double curvature = curvatureToGoal();
        double c = 2/(tankRobot.getLateralWheelDistance()*curvature);
        double velLeftToRightRatio = -( c + 1 ) / (1-c);
        double velRightToLeftRatio = 1/velLeftToRightRatio;
        double score = -Integer.MAX_VALUE;
        Vector bestVector = null;
        //TODO: fix clumsy way of optimizing :(

        double v_lMax = tankRobot.getV_lMax();
        double v_rMax = tankRobot.getV_rMax();
        double v_lMin = tankRobot.getV_lMin();
        double v_rMin = tankRobot.getV_rMin();

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

        rotVelocity = (bestVector.get(1) - bestVector.get(0)) / tankRobot.getLateralWheelDistance();

        // Note this can be negative

        r = 1 / curvature;

        currentAngle = tankRobot.getAngle();
        double thetaToRotate = MathUtils.Arithmetic.sign(rotVelocity)*Math.atan(goalPoint.get(1) / (Math.abs(r)-goalPoint.get(0)));

        Function<Double,Vector> estimatePositionFromDTheta = dTheta -> {
            double dxRelative = -r*(1-Math.cos(-dTheta));
            double dyRelative = -r*Math.sin(-dTheta);
            Vector dRelativeVector = new Vector(dxRelative, dyRelative);
            Vector rotated = MathUtils.LinearAlgebra.rotate2D(dRelativeVector, currentAngle);
            Vector toReturn = rotated.add(usedEstimatedLocation);
            return toReturn;
        };

        estimateHeadingFromTime = time-> {
            double heading = currentAngle + rotVelocity*time;
            return heading;
        };

        esimatePositionFromRotation = angle -> {
            double dTheta = angle - currentAngle;
            return estimatePositionFromDTheta.test(dTheta);
        };

        estimatedTime = thetaToRotate / rotVelocity;

        timeToPosition = time -> {
            double dTheta = time * rotVelocity;
            return estimatePositionFromDTheta.test(dTheta);
        };

        return bestVector;
    }

    private Vector absoluteToRelativeCoord(Vector waypoint){
        if(waypoint.dimensions() != 2)
            throw new IllegalArgumentException("Must be in R2");
        Vector coordDif = waypoint.clone().subtractBy(tankRobot.getEsimatedLocation());
        rotatedAngle = tankRobot.getAngle();
        Vector toReturn = MathUtils.LinearAlgebra.rotate2D(coordDif, -rotatedAngle);
        return toReturn;
    }

    public Vector getGoalPointAbsolute() {
        return goalPointAbsolute;
    }
}
