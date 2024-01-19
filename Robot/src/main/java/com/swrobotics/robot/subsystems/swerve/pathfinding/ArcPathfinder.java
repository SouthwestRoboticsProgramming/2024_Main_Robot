package com.swrobotics.robot.subsystems.swerve.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.swrobotics.messenger.client.MessageReader;
import com.swrobotics.messenger.client.MessengerClient;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public final class ArcPathfinder implements Pathfinder {
    private enum PathStatus {
        READY,
        IMPOSSIBLE,
        TIMED_OUT
    }

    private static final String MSG_SET_ENDPOINTS = "Pathfinder:SetEndpoints";
    private static final String MSG_PATH = "Pathfinder:Path";

    private static final double CORRECT_TARGET_TOL = 0.1524 + 0.1;
    private static final double TIMEOUT = 1; // Seconds

    private final MessengerClient msg;
    private final Pathfinder fallback;

    private Translation2d start, goal;
    private boolean pathRequestPending;
    private double pathRequestTimestamp;

    private boolean newPathAvail;
    private List<Translation2d> pathBezier;
    private PathStatus status;

    public ArcPathfinder(MessengerClient msg) {
        this.msg = msg;
        msg.addHandler(MSG_PATH, this::onPath);

        newPathAvail = false;
        start = goal = new Translation2d();
        pathRequestPending = false;

        fallback = new LocalADStar();
    }

    private void markNewPath(PathStatus status) {
        newPathAvail = true;
        pathRequestPending = false;
        this.status = status;
    }

    private void onPath(String type, MessageReader reader) {
        System.out.println("Path????");
        boolean pathValid = reader.readBoolean();
        if (!pathValid) {
            System.out.println("Invalid :(");
            markNewPath(PathStatus.IMPOSSIBLE);
            return;
        }

        int count = reader.readInt();
        if (count % 3 != 1) {
            System.out.println("its goofy cant do it :(");
            // Something went wrong, it wasn't a valid Bezier path
            return;
        }

        List<Translation2d> path = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double x = reader.readDouble();
            double y = reader.readDouble();

            path.add(new Translation2d(x, y));
        }

        // Check if path is to correct target
        // In case of latency returning path for previous target
        Translation2d lastPoint = path.get(path.size() - 1);
        if (lastPoint.minus(goal).getNorm() > CORRECT_TARGET_TOL) {
            System.out.println("its going wrong place????");
            return;
        }

        pathBezier = path;
        markNewPath(PathStatus.READY);
        System.out.println("path is yes");
        System.out.println(pathBezier);
    }

    @Override
    public boolean isNewPathAvailable() {
        // If pathfinder isn't responding, give up after timeout elapses
        if (pathRequestPending && Timer.getFPGATimestamp() - pathRequestTimestamp > TIMEOUT) {
            newPathAvail = true;
            pathRequestPending = false;
            status = PathStatus.TIMED_OUT;

            // Try to get path from the fallback pathfinder
            fallback.setStartPosition(start);
            fallback.setGoalPosition(goal);
        }

        if (status == PathStatus.TIMED_OUT || status == PathStatus.IMPOSSIBLE)
            return fallback.isNewPathAvailable();

        return newPathAvail;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        if (!newPathAvail && status != PathStatus.TIMED_OUT)
            return null;
        newPathAvail = false;

        return switch (status) {
            case READY -> new PathPlannerPath(pathBezier, constraints, goalEndState);
            // Make a dummy path that goes straight from start to goal
            case IMPOSSIBLE, TIMED_OUT -> fallback.getCurrentPath(constraints, goalEndState);
        };
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        start = startPosition;
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        goal = goalPosition;
        msg.prepare(MSG_SET_ENDPOINTS)
                .addDouble(start.getX()).addDouble(start.getY())
                .addDouble(goalPosition.getX()).addDouble(goalPosition.getY())
                .send();
        pathRequestPending = true;
        pathRequestTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        throw new UnsupportedOperationException();
    }
}
