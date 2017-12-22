package com.github.ultrax3.purepursuit;

import processing.core.PApplet;

import java.util.Arrays;
import java.util.List;

public class PurePursuitTest extends PApplet{

    /*
    Sources:
    - https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    - https://www.usna.edu/Users/cs/crabbe/SI475/current/mob-kin/mobkin.pdf
    - https://www.mathworks.com/help/robotics/ug/pure-pursuit-controller.html
    - http://mathworld.wolfram.com/Circle-LineIntersection.html
    - https://www.youtube.com/watch?v=qG70QJJ8Qz8
    - https://www.youtube.com/watch?v=9fzzp6oxid4

    Explanation for some equations:
    https://www.sharelatex.com/read/khftjrmkfzqy

     */

    List<Vector> goals = Arrays.asList(
            new Vector(0,0),
            new Vector(2,2),
            new Vector(10,1)
    );

    TankRobot tankRobot = new TankRobot(1, new Vector(0, 0), 0, -1, 1,
            -1, 1);

    public static void main(String[] args) {

        PApplet.main("com.github.ultrax3.purepursuit.PurePursuitTest");
        new PurePursuitTest();

        /*
        while (true){
            tankRobot.update();
            System.out.println(tankRobot.getEsimatedLocation());
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        */
    }

    public void settings(){
        size(400,400);
    }

    public void setup(){
        settings();
        fill(0,0,0);
        frameRate(10);
        PurePursuitMovementStrategy movementStrategy = new PurePursuitMovementStrategy(tankRobot, goals,1);
        tankRobot.setMovementStrategy(movementStrategy);
    }

    public void draw() {
        tankRobot.update();
        background(255,255,255);
        fill(0,0,0);
        for(Vector vector : goals){
            stroke(0, 0, 255);
            ellipse(getX(vector.get(0)),getY(vector.get(1)),2,2);
        }

        TankMovementStrategy movementStrategy = tankRobot.getMovementStrategy();
        stroke(0, 0, 0);
        float drawX = (float) (10 * tankRobot.getEsimatedLocation().get(0)) + 200;
        float drawY = (float) (10 * tankRobot.getEsimatedLocation().get(1)) + 200;
        Vector lineAddPoint = MathUtils.LinearAlgebra.rotate2D(new Vector(0, 30), tankRobot.getAngle());
        super.ellipse(drawX, 400-drawY,10,10);
        super.line(drawX, 400-drawY,(float)(drawX+lineAddPoint.get(0)),400-(float)(drawY+lineAddPoint.get(1)));
        stroke(0,255,0);
        noFill();
        super.ellipse(drawX, 400-drawY,20,20);
        if (movementStrategy instanceof PurePursuitMovementStrategy){
            fill(0,0,0);
            stroke(255, 0, 0);
            PurePursuitMovementStrategy purePursuitMovementStrategy = (PurePursuitMovementStrategy) movementStrategy;
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
