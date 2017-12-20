package com.github.ultrax3.purepursuit;

import java.util.List;

public class PurePursuitMovementStrategy implements TankMovementStrategy{

    private final List<Vector> pathPoints;
    private Vector goalPoint;
    private final TankRobot tankRobot;
    private double r;
    private double rotVelocity;
    private double lookAheadDistance;

    int lastPath = 0;

    Function<Double,Vector> esimatePositionFromRotation;
    private double estimatedTime;
    private Function<Double, Vector> timeToPosition;
    private float rotatedAngle;
    private Vector usedEstimatedLocation;

    public PurePursuitMovementStrategy(TankRobot tankRobot, List<Vector> goals, double lookAheadDistance){
        this.pathPoints = goals;
        this.tankRobot = tankRobot;
        this.lookAheadDistance = lookAheadDistance;
    }

    public Function<Double, Vector> estimateTimeToPosition() {
        return timeToPosition;
    }

    public Function<Double, Vector> getEsimatePositionFromRotation() {
        return esimatePositionFromRotation;
    }

    @Override
    public void start() {

    }

    public void update() {
        for(int i =lastPath; i< pathPoints.size()-1; i++){
            usedEstimatedLocation = tankRobot.getEsimatedLocation();
            Vector[] vectors = MathUtils.Geometry.circleLineIntersect(pathPoints.get(i), pathPoints.get(i + 1), usedEstimatedLocation, lookAheadDistance);
            if(vectors.length != 0){
                lastPath = i;
                if(vectors.length == 2) {
                    if (vectors[0].clone().subtractBy(pathPoints.get(i + 1)).getMagnitudeSquared() <
                            vectors[1].clone().subtractBy(pathPoints.get(i + 1)).getMagnitudeSquared()) {
                        goalPoint = absoluteToRelativeCoord(vectors[0]);
                    } else {
                        goalPoint = absoluteToRelativeCoord(vectors[1]);
                    }
                }
                else{
                    goalPoint = absoluteToRelativeCoord(vectors[0]);
                }
                break;
            }
        }
        Vector wheelTangentialVelocity = getWheelTangentialVelocity();
        tankRobot.setLeftTreadVelocity(wheelTangentialVelocity.get(0));
        tankRobot.setRightTreadVelocity(wheelTangentialVelocity.get(1));
    }

    private double curvatureToGoal(){
        double lSquared = goalPoint.getMagnitudeSquared(); // x^2 + y^2 = l^2 (length)
        return -2*goalPoint.get(0)/lSquared;
    }

    private Vector getWheelTangentialVelocity(){
        double curvature = curvatureToGoal();
        double c = 2/(tankRobot.getLateralWheelDistance()*curvature); //TODO: !!! this should be 2/(lateralWheelDistance*curvature), but this doesn't return consistent answers
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

        final double currentAngle = tankRobot.getAngle();
        double thetaToRotate = MathUtils.Arithmetic.sign(rotVelocity)*Math.atan(goalPoint.get(1) / (Math.abs(r)-goalPoint.get(0)));

        Function<Double,Vector> estimatePositionFromDTheta = dTheta -> {
            double dxRelative = -Math.abs(r)*Math.cos(-dTheta) + Math.abs(r);
            double dyRelative = Math.abs(r)*Math.sin(-dTheta);

            Vector dPosAbsolute = MathUtils.LinearAlgebra.rotate2D(new Vector(dxRelative,dyRelative),-rotatedAngle);
            Vector toReturn = usedEstimatedLocation.clone().add(dPosAbsolute);
            return toReturn;
        };

        esimatePositionFromRotation = angle -> {
            double dTheta = angle - currentAngle;
            return estimatePositionFromDTheta.test(dTheta);
        };

        // tangentialSpeed = Math.abs(r*rotVelocity);
        // arclength = distanceRotated *r;
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
        return MathUtils.LinearAlgebra.rotate2D(coordDif, rotatedAngle);
    }

}
