package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoDisBlue")
public class AutoDisBlue extends LinearOpMode {

    private String selectedAuto = null;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //sensor finds something in middle
        TrajectorySequence middleBF = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                //.waitSeconds(3)
                .lineTo(new Vector2d(-36, 14),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .waitSeconds(.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-.2))
                .waitSeconds(.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-36, 8),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(36, 8),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
        //(48)
                .lineTo(new Vector2d(60, 20),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .build();

        // TODO Stop the secret cat invasion

        //strafe
        TrajectorySequence strafeBF = drive.trajectorySequenceBuilder(new Pose2d(-36, 63.50, Math.toRadians(-85.00)))
                .strafeRight(13)
                .build();

        //sensor finds something in left (facing red)
        TrajectorySequence leftBF = drive.trajectorySequenceBuilder(new Pose2d(-38, 63.50, Math.toRadians(-85.00)))
                .splineToLinearHeading(new Pose2d(-35.00, 35.00, Math.toRadians(180.00)), Math.toRadians(0.00),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .lineTo(new Vector2d(-30,35),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .lineTo(new Vector2d(-37,35),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.2))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-40, 35))
                .lineTo(new Vector2d(-40.00, 33.00))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-37.00, 12.00))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(38.00, 12.00),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .lineTo(new Vector2d(60, 25),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .build();


        // TODO Take the lazers away from the cats(exept Izzy)(esspecially Loki)

        //sensor finds something in right (facing red)
        TrajectorySequence rightBF = drive.trajectorySequenceBuilder(new Pose2d(-46, 63.50, Math.toRadians(-85.00)))
                .lineTo(new Vector2d(-46, 20),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-.4))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-36, 10))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(30, 10),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .lineTo(new Vector2d(60, 25),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .build();

        while (opModeInInit()) {
            drive.planeServo.setPosition(0.6);
            if (gamepad1.y) {
                selectedAuto = "BF";
                drive.setPoseEstimate(middleBF.start());
                drive.planeServo.setPosition(0.6);
                telemetry.addData("H", selectedAuto);
                telemetry.addData("disSen", drive.disSen.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        waitForStart();

        if (drive.disSen.getDistance(DistanceUnit.INCH) > 24 && drive.disSen.getDistance(DistanceUnit.INCH) < 30) {
            switch (selectedAuto) {
                case "BF":
                    drive.followTrajectorySequence(middleBF);
                    break;
            }
        } else{
            switch (selectedAuto) {
                case "BF":
                    drive.followTrajectorySequence(strafeBF);

                    if (drive.disSen.getDistance(DistanceUnit.INCH) > 16 && drive.disSen.getDistance(DistanceUnit.INCH) < 30) {
                        switch(selectedAuto){
                            case "BF":
                                drive.followTrajectorySequence(rightBF);
                                break;
                        }
                    } else {
                        switch(selectedAuto){
                            case "BF":
                                drive.followTrajectorySequence(leftBF);
                        }
                    }
            }
        }

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.clearAll();
            telemetry.addData("Current Auto", selectedAuto);
            telemetry.addLine();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addLine();
            telemetry.addData("DisSensor", drive.disSen.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    public void placePixels() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        timer.reset();
        //16
        //1.695

        drive.slideMotor.setTargetPosition(4400);
        drive.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double slidePosition = drive.slideMotor.getCurrentPosition();
        double currentVoltage = drive.potentiometer.getVoltage();

        while (timer.seconds() < 4){
            drive.slideMotor.setPower(1);
        }

        while (timer.seconds() < 5){
            drive.slideMotor.setPower(0);
            drive.slideMotor.setTargetPosition(0);
        }

        while (timer.seconds() < 8){
            drive.gripServo.setPower(-1);
        }

        while (timer.seconds() < 10){
            drive.gripServo.setPower(0);
            drive.positionServo.setPosition(0);
        }

        while (timer.seconds() < 12.1){
            drive.gripServo.setPower(1);
        }

        while (timer.seconds() < 16.1){
            drive.gripServo.setPower(0);
            drive.slideMotor.setPower(-1);
            drive.positionServo.setPosition(1);
        }

        while (timer.seconds() < 17.1){
            drive.slideMotor.setPower(0);
        }

        telemetry.addData("slidePos", slidePosition);
        telemetry.addData("currentVolts", currentVoltage);
        telemetry.update();

    }
    }