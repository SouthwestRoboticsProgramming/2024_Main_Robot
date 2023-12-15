package com.swrobotics.robot.control;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.lib.input.XboxController;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.subsystems.swerve.pathfinding.CircleShape;
import com.swrobotics.robot.subsystems.swerve.pathfinding.ThetaStarPathfinder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DynamicObstacleBot extends SubsystemBase {
    private final XboxController controller = new XboxController(1, 0.2);
    private Pose2d pose = new Pose2d(1,1, new Rotation2d());
    private final RobotContainer robot;

    public DynamicObstacleBot(RobotContainer robot) {
        this.robot = robot;
    }

    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void periodic() {
        pose = pose.plus(new Transform2d(-controller.getLeftStick().y, -controller.getLeftStick().x, new Rotation2d()));
        // ThetaStarPathfinder.getInstance().setDynamicShapes(
        //     Arrays.asList(new CircleShape(pose.getX(), pose.getY(), 1)), robot.drive.getEstimatedPose().getTranslation());

        Logger.recordOutput("Dynamic Obstacle Pose", getPose());
    }
}
