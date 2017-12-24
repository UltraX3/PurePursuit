package com.github.ultrax3.purepursuit;

import processing.core.PApplet;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class PurePursuitTest extends PApplet{

    /*
    Sources:
    - Paper on PurePursuit: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    - Paper on Mobile Robot Kinematics: https://www.usna.edu/Users/cs/crabbe/SI475/current/mob-kin/mobkin.pdf
    - Explanation of PurePursuit: https://www.mathworks.com/help/robotics/ug/pure-pursuit-controller.html
    - Explanation of circle-line intersection: http://mathworld.wolfram.com/Circle-LineIntersection.html
    - PurePursuit example video: https://www.youtube.com/watch?v=qG70QJJ8Qz8
    - MIT PurPursuit example video https://www.youtube.com/watch?v=9fzzp6oxid4
    -

    Explanation for some equations:
    https://www.sharelatex.com/read/khftjrmkfzqy

     */

    List<Vector> goals = new ArrayList<>();

    TankRobot tankRobot = new TankRobot(1, new Vector(0, 0), 0, -1, 1,
            -1, 1);

    public PurePursuitTest() {
        goals.add(new Vector(0,0));
        Random random = new Random();
        for(int i =0; i< 10; i++){
            Vector vector = new Vector(15-random.nextDouble()*30,15-random.nextDouble()*30);
            goals.add(vector);
        }
    }

    public static void main(String[] args) {



        PApplet.main("com.github.ultrax3.purepursuit.PurePursuitTest");
        new PurePursuitTest();

    }

    public void settings(){
        size(400,400);
    }

    public void setup(){
        settings();
        fill(0,0,0);
        frameRate(60);
        PurePursuitMovementStrategy movementStrategy = new PurePursuitMovementStrategy(tankRobot, goals,1);
        tankRobot.setMovementStrategy(movementStrategy);
    }

    public void draw() {
        tankRobot.update();
        background(255,255,255);
        fill(0,0,0);
        stroke(0, 0, 255);
        strokeWeight(0.2F);
        for (int i = 0; i < goals.size()-1; i++) {
            Vector a = goals.get(i);
            Vector b = goals.get(i+1);
            line(getX(a.get(0)),getY(a.get(1)),getX(b.get(0)),getY(b.get(1)));
        }
        for(Vector vector : goals){
            stroke(0, 0, 255);
            ellipse(getX(vector.get(0)),getY(vector.get(1)),2,2);
        }
        strokeWeight(1);

        TankMovementStrategy movementStrategy = tankRobot.getMovementStrategy();
        stroke(0, 0, 0);
        float drawX = (float) (10 * tankRobot.getEsimatedLocation().get(0)) + 200;
        float drawY = (float) (10 * tankRobot.getEsimatedLocation().get(1)) + 200;
        Vector lineAddPoint = MathUtils.LinearAlgebra.rotate2D(new Vector(0, 30), tankRobot.getAngle());
        super.ellipse(drawX, 400-drawY,10,10);
        super.line(drawX, 400-drawY,(float)(drawX+lineAddPoint.get(0)),400-(float)(drawY+lineAddPoint.get(1)));
        if (movementStrategy instanceof PurePursuitMovementStrategy){
            PurePursuitMovementStrategy purePursuitMovementStrategy = (PurePursuitMovementStrategy) movementStrategy;
            stroke(0,255,0);
            noFill();
            super.ellipse(drawX, 400-drawY, (float) (purePursuitMovementStrategy.getLookAheadDistance()*20),(float) (purePursuitMovementStrategy.getLookAheadDistance()*20));
            fill(0,0,0);
            stroke(255, 0, 0);
            Vector goalPoint = purePursuitMovementStrategy.getGoalPointAbsolute();
            ellipse(getX(goalPoint.get(0)),getY(goalPoint.get(1)),5,5);
            for (Vector intersection : purePursuitMovementStrategy.intersections) {
                stroke(0, 0, 0);
                noFill();
                ellipse(getX(intersection.get(0)),getY(intersection.get(1)),5,5);
            }
        }
    }

    float getX(double x){
        return (float) (10 * x) + 200;
    }

    float getY(double y){
        return (float) 400 - ((float) (10 * y) + 200);
    }

    //void drawDot(double x, double y){
    //
    //    super.ellipse(xCoord, yCoord,2,2);
    //}
}
