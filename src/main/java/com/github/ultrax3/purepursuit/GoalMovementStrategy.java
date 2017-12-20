package com.github.ultrax3.purepursuit;

import java.util.List;

public interface GoalMovementStrategy extends TankMovementStrategy {
    List<Vector> getGoalPoints();
}
