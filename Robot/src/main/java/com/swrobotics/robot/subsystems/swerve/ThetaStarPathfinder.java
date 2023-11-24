package com.swrobotics.robot.subsystems.swerve;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.swrobotics.messenger.client.MessageReader;
import com.swrobotics.messenger.client.MessengerClient;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public final class ThetaStarPathfinder implements Pathfinder {
    private static final String MSG_SET_POS = "Pathfinder:SetPos";
    private static final String MSG_SET_GOAL = "Pathfinder:SetGoal";
    private static final String MSG_PATH = "Pathfinder:Path";

    private static final double CORRECT_TARGET_TOL = 0.1524 + 0.1;

    // Directly taken from PPLib
    private static final double SMOOTHING_ANCHOR_PCT = 0.8;
    private static final double SMOOTHING_CONTROL_PCT = 0.33;

    private final MessengerClient msg;

    private Translation2d start, goal;

    private boolean newPathAvail;
    private List<Translation2d> pathBezier;

    public ThetaStarPathfinder(MessengerClient msg) {
        this.msg = msg;
        msg.addHandler(MSG_PATH, this::onPath);

        newPathAvail = false;
        start = goal = new Translation2d();
    }

    private void onPath(String type, MessageReader reader) {
        System.out.println("Path received");
        boolean pathValid = reader.readBoolean();
        if (!pathValid) {
            // FIXME: If this happens, the path following command never ends
            //  this effectively disables the drive base until robot is disabled
            System.out.println("It was invalid :(");
            return;
        }

        int count = reader.readInt();
        if (count < 2) {
            System.out.println("Not enough points???");
            return;
        }

        List<Translation2d> pathPoints = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double x = reader.readDouble();
            double y = reader.readDouble();

            pathPoints.add(new Translation2d(x, y));
        }
        System.out.println("Path points: " + pathPoints);

        // Check if path is to correct target
        // In case of latency returning path for previous target
        Translation2d lastPoint = pathPoints.get(pathPoints.size() - 1);
        if (lastPoint.minus(goal).getNorm() > CORRECT_TARGET_TOL) {
            System.out.println("Wrong target... old path? want to go to " + goal);
            return;
        }

        // Replace start and end with actual points
        pathPoints.set(0, start);
        pathPoints.set(pathPoints.size() - 1, goal);

        // Calculate Bezier points
        // Taken from PPLib LocalADStar#extractPath()
        pathBezier = new ArrayList<>();
        pathBezier.add(pathPoints.get(0));
        pathBezier.add(pathPoints
                .get(1)
                .minus(pathPoints.get(0))
                .times(SMOOTHING_CONTROL_PCT)
                .plus(pathPoints.get(0)));
        for (int i = 1; i < pathPoints.size() - 1; i++) {
            Translation2d last = pathPoints.get(i - 1);
            Translation2d current = pathPoints.get(i);
            Translation2d next = pathPoints.get(i + 1);

            Translation2d anchor1 = current.minus(last).times(SMOOTHING_ANCHOR_PCT).plus(last);
            Translation2d anchor2 = current.minus(next).times(SMOOTHING_ANCHOR_PCT).plus(next);

            double controlDist = anchor1.getDistance(anchor2) * SMOOTHING_CONTROL_PCT;

            Translation2d prevControl1 = last.minus(anchor1).times(SMOOTHING_CONTROL_PCT).plus(anchor1);
            Translation2d nextControl1 =
                    new Translation2d(controlDist, anchor1.minus(prevControl1).getAngle()).plus(anchor1);

            Translation2d prevControl2 =
                    new Translation2d(controlDist, anchor2.minus(next).getAngle()).plus(anchor2);
            Translation2d nextControl2 = next.minus(anchor2).times(SMOOTHING_CONTROL_PCT).plus(anchor2);

            pathBezier.add(prevControl1);
            pathBezier.add(anchor1);
            pathBezier.add(nextControl1);

            pathBezier.add(prevControl2);
            pathBezier.add(anchor2);
            pathBezier.add(nextControl2);
        }
        pathBezier.add(
                pathPoints
                        .get(pathPoints.size() - 2)
                        .minus(pathPoints.get(pathPoints.size() - 1))
                        .times(SMOOTHING_CONTROL_PCT)
                        .plus(pathPoints.get(pathPoints.size() - 1)));
        pathBezier.add(pathPoints.get(pathPoints.size() - 1));

        newPathAvail = true;
        System.out.println("Bezierified into " + pathBezier);
    }

    @Override
    public boolean isNewPathAvailable() {
        return newPathAvail;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        if (!newPathAvail)
            return null;
        newPathAvail = false;
        return new PathPlannerPath(pathBezier, constraints, goalEndState);
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        start = startPosition;
        msg.prepare(MSG_SET_POS).addDouble(startPosition.getX()).addDouble(startPosition.getY()).send();
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        goal = goalPosition;
        msg.prepare(MSG_SET_GOAL).addDouble(goalPosition.getX()).addDouble(goalPosition.getY()).send();
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        // Not supported (yet)
    }
}
