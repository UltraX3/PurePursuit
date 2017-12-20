package com.github.ultrax3.purepursuit;

public class TankRobot implements TankMoveable{

    private final double lateralWheelDistance;
    private final double v_rMax,v_lMax,v_lMin, v_rMin;
    private Vector estimatedLocation;
    private final float angle;
    private TankMovementStrategy movementStrategy;
    private long lastUpdateNano = -1;

    // TODO: fix weird angle

    public TankRobot(double lateralWheelDistance, Vector location, float angle, double v_lMin, double v_lMax, double v_rMin, double v_rMax){
        this.lateralWheelDistance = lateralWheelDistance;
        this.v_lMax = v_lMax;
        this.v_rMax = v_rMax;
        this.v_lMin = v_lMin;
        this.v_rMin = v_rMin;
        this.estimatedLocation = location;
        this.angle = angle;
    }

    public Vector getEsimatedLocation() {
        return estimatedLocation;
    }

    public float getAngle() {
        return angle;
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

    public void setMovementStrategy(TankMovementStrategy movementStrategy){
        this.movementStrategy = movementStrategy;
    }

    public void update(){
        // long newNano = System.nanoTime();
        if(lastUpdateNano != -1 && movementStrategy instanceof PurePursuitMovementStrategy){
            PurePursuitMovementStrategy purePursuitMovementStrategy = (PurePursuitMovementStrategy) this.movementStrategy;
            estimatedLocation = purePursuitMovementStrategy.estimateTimeToPosition().test(0.05);
            //estimatedLocation = purePursuitMovementStrategy.estimateTimeToPosition().test((newNano-lastUpdateNano)/1000000000.0);
        }
        movementStrategy.update();
        lastUpdateNano = 1;
        //lastUpdateNano = newNano;
    }

    public double getLateralWheelDistance() {
        return lateralWheelDistance;
    }

    @Override
    public void setLeftTreadVelocity(double velocity) {

    }

    @Override
    public void setRightTreadVelocity(double velocity) {

    }
}
