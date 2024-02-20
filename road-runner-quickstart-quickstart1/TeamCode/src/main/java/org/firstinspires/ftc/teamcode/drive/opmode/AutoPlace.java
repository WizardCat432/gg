package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.RenderNode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Place")
public class AutoPlace extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d(-40, 63.50, Math.toRadians(-85.00)))
                .splineToSplineHeading(new Pose2d(-29.72, 37.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, 38.41))
                .lineToSplineHeading(new Pose2d(48.79, 37.84, Math.toRadians(180.00)))
                .build();

        Trajectory trajectoryServo = drive.trajectoryBuilder(trajectoryForward.end())

                        .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryForward);
        }
    }
}


