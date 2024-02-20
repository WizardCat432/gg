package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Park")
public class AutoParking extends LinearOpMode {

    private String selectedAuto = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence bfpPath = drive.trajectorySequenceBuilder(new Pose2d(-40, 63.50, Math.toRadians(270)))
                .splineTo(new Vector2d(-39.35, 38.22), Math.toRadians(-85.52))
                .splineTo(new Vector2d(-14.63, 9.72), Math.toRadians(4.22))
                .splineTo(new Vector2d(24.63, 10.10), Math.toRadians(0.28))
                .splineTo(new Vector2d(62.19, 11.80), Math.toRadians(3.01))
                .build();


        TrajectorySequence bcpPath = drive.trajectorySequenceBuilder(new Pose2d(40, 63.50, Math.toRadians(270.00)))
                .lineTo(new Vector2d(65, 60))
                .build();

        TrajectorySequence rfpPath = drive.trajectorySequenceBuilder(new Pose2d(-40.00, -67.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-39.35, -38.22), Math.toRadians(445.52))
                .splineTo(new Vector2d(-14.63, -9.72), Math.toRadians(355.78))
                .splineTo(new Vector2d(24.63, -10.10), Math.toRadians(359.72))
                .splineTo(new Vector2d(62.19, -11.80), Math.toRadians(356.99))
                .build();

        TrajectorySequence rcpPath = drive.trajectorySequenceBuilder(new Pose2d(40, -63.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(65, -60))
                .build();

        telemetry.addLine("Please choose from the following Autonomous programs:");
        telemetry.addData("A", "Blue Park (Close)");
        telemetry.addData("B", "Blue Park (Far)");
        telemetry.addData("X", "Red Park (Close)");
        telemetry.addData("Y", "Red Park (Far)");

        telemetry.update();

        while(opModeInInit()) {
            if (gamepad1.a) {
                selectedAuto = "bcp";

                drive.setPoseEstimate(bcpPath.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Blue Park (Close)");
                telemetry.update();
            } else if (gamepad1.b) {
                selectedAuto = "bfp";

                drive.setPoseEstimate(bfpPath.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Blue Park (Far)");
                telemetry.update();
            } else if (gamepad1.x) {
                selectedAuto = "rcp";

                drive.setPoseEstimate(rcpPath.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Red Park (Close)");
                telemetry.update();
            } else if (gamepad1.y) {
                selectedAuto = "rfp";

                drive.setPoseEstimate(rfpPath.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Red Park (Far)");
                telemetry.update();
            }
        }

        waitForStart();

        switch (selectedAuto) {
            case "bcp":
                drive.followTrajectorySequence(bcpPath);
                break;
            case "bfp":
                drive.followTrajectorySequence(bfpPath);
                break;
            case "rcp":
                drive.followTrajectorySequence(rcpPath);
                break;
            case "rfp":
                drive.followTrajectorySequence(rfpPath);
                break;
        }

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.clearAll();
        telemetry.addData("Current Auto", selectedAuto);
        telemetry.addLine();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

    }
}
