package com.swrobotics.robot.subsystems.swerve.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.swrobotics.messenger.client.MessageBuilder;
import com.swrobotics.messenger.client.MessageReader;
import com.swrobotics.messenger.client.MessengerClient;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public final class ThetaStarPathfinder implements Pathfinder {
    private static ThetaStarPathfinder INSTANCE;

    public static ThetaStarPathfinder getInstance() {
        return INSTANCE;
    }

    private enum PathStatus {
        READY,
        ALREADY_THERE,
        IMPOSSIBLE,
        TIMED_OUT
    }

    private static final String MSG_SET_ENDPOINTS = "Pathfinder:SetEndpoints";
    private static final String MSG_PATH = "Pathfinder:Path";
    private static final String MSG_SET_DYN_SHAPES = "Pathfinder:SetDynamicShapes";

    private static final double CORRECT_TARGET_TOL = 0.1524 + 0.1;
    private static final double TIMEOUT = 1; // Seconds

    // Directly taken from PPLib
    private static final double SMOOTHING_ANCHOR_PCT = 0.8;
    private static final double SMOOTHING_CONTROL_PCT = 0.33;

    private final MessengerClient msg;
    private final Pathfinder fallback;

    private Translation2d start, goal;
    private boolean pathRequestPending;
    private double pathRequestTimestamp;

    private boolean newPathAvail;
    private List<Translation2d> pathBezier;
    private PathStatus status;

    public ThetaStarPathfinder(MessengerClient msg) {
        if (INSTANCE != null)
            throw new IllegalStateException("ThetaStarPathfinder already initialized");
        INSTANCE = this;

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
        boolean pathValid = reader.readBoolean();
        if (!pathValid) {
            markNewPath(PathStatus.IMPOSSIBLE);
            return;
        }

        int count = reader.readInt();
        if (count < 2) {
            // This only happens if start and end are the same point, in which
            // case we can skip the path following since we're already there
            markNewPath(PathStatus.ALREADY_THERE);
            return;
        }

        List<Translation2d> pathPoints = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double x = reader.readDouble();
            double y = reader.readDouble();

            pathPoints.add(new Translation2d(x, y));
        }

        // Check if path is to correct target
        // In case of latency returning path for previous target
        Translation2d lastPoint = pathPoints.get(pathPoints.size() - 1);
        if (lastPoint.minus(goal).getNorm() > CORRECT_TARGET_TOL) {
            return;
        }

        // Replace start and end with actual points
        pathPoints.set(0, start);
        pathPoints.set(pathPoints.size() - 1, goal);

        pathBezier = generateBezier(pathPoints);
        markNewPath(PathStatus.READY);
    }

    private List<Translation2d> generateBezier(List<Translation2d> pathPoints) {
        // Calculate Bezier points
        // Taken from PPLib LocalADStar#extractPath()
        List<Translation2d> pathBezier = new ArrayList<>();
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
        return pathBezier;
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

        if (status == PathStatus.TIMED_OUT)
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
            case ALREADY_THERE, IMPOSSIBLE -> new PathPlannerPath(generateBezier(List.of(start, goal)), constraints, goalEndState);
            case TIMED_OUT -> fallback.getCurrentPath(constraints, goalEndState);
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

    public void setDynamicShapes(List<Shape> shapes, Translation2d currentRobotPos) {
        MessageBuilder builder = msg.prepare(MSG_SET_DYN_SHAPES);
        builder.addInt(shapes.size());
        for (Shape obs : shapes) {
            obs.write(builder);
        }

        builder.addDouble(currentRobotPos.getX());
        builder.addDouble(currentRobotPos.getY());
        builder.send();
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        List<Shape> shapes = new ArrayList<>();
        for (Pair<Translation2d, Translation2d> axisAlignedBB : obs) {
            Translation2d to = axisAlignedBB.getFirst();
            Translation2d from = axisAlignedBB.getSecond();

            Translation2d center = to.plus(from.minus(to).times(0.5));
            double width = Math.abs(from.getX() - to.getX());
            double height = Math.abs(from.getY() - to.getY());

            shapes.add(new RectangleShape(
                    center.getX(), center.getY(),
                    width, height,
                    new Rotation2d(0),
                    false
            ));
        }

        setDynamicShapes(shapes, currentRobotPos);
    }
}
