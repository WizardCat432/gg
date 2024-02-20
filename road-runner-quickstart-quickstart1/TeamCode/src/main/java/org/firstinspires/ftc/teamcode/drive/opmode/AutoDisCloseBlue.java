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

@Autonomous(name="AutoDisCloesBlue")
public class AutoDisCloseBlue extends LinearOpMode {

    private String selectedAuto = null;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //sensor finds something in middle
        TrajectorySequence middleRF = drive.trajectorySequenceBuilder(new Pose2d(11, 64, Math.toRadians(-90)))
                .lineTo(new Vector2d(11, 38),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .turn(Math.toRadians(190))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-.2))
                .waitSeconds(.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .waitSeconds(.5)
                .lineTo(new Vector2d(15, 68),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .lineTo(new Vector2d(58, 68),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .build();

        //strafe
        TrajectorySequence strafeRF = drive.trajectorySequenceBuilder(new Pose2d(11, 64, Math.toRadians(-90)))
                .strafeLeft(13)
                .build();

        //sensor finds something in left (facing blue)
        TrajectorySequence leftRF = drive.trajectorySequenceBuilder(new Pose2d(23, 63.50, Math.toRadians(-90)))
                .lineTo(new Vector2d(23, 44),
                     SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                     SampleMecanumDrive.getAccelerationConstraint(52.567))
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> drive.intakeMotor.setPower(-.2))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> drive.intakeMotor.setPower(0))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(23, 65),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .lineTo(new Vector2d(60, 65),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .build();

        //sensor finds something in right (facing blue)
        TrajectorySequence rightRF = drive.trajectorySequenceBuilder(new Pose2d(23, 63.50, Math.toRadians(90)))
                .lineTo(new Vector2d(23, 38),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(-3, 38),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .lineTo(new Vector2d(12, 38),
                        SampleMecanumDrive.getVelocityConstraint(6, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .waitSeconds(.5)
                .addTemporalMarker(()-> drive.intakeMotor.setPower(-.2))
                .waitSeconds(.5)
                .addTemporalMarker(()-> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(23, 30))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(23, 64),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .lineTo(new Vector2d(60, 64),
                        SampleMecanumDrive.getVelocityConstraint(24, Math.toRadians(245.179), 15.82),
                        SampleMecanumDrive.getAccelerationConstraint(52.567))
                .build();

        while (opModeInInit()) {
            drive.planeServo.setPosition(0.6);
            if (gamepad1.y) {
                selectedAuto = "RF";
                drive.setPoseEstimate(middleRF.start());
                telemetry.addData("H", selectedAuto);
                telemetry.addData("disSen", drive.disSen.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        waitForStart();

        if (drive.disSen.getDistance(DistanceUnit.INCH) > 24 && drive.disSen.getDistance(DistanceUnit.INCH) < 30) {
            switch (selectedAuto) {
                case "RF":
                    drive.followTrajectorySequence(middleRF);
                    //placePixels();
                    break;
            }
        } else {
            switch (selectedAuto) {
                case "RF":
                    drive.slideMotor.setTargetPosition(2000);
                    drive.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    drive.followTrajectorySequence(strafeRF);

                    if (drive.disSen.getDistance(DistanceUnit.INCH) > 14 && drive.disSen.getDistance(DistanceUnit.INCH) < 30) {
                        switch (selectedAuto) {
                            case "RF":
                                drive.followTrajectorySequence(leftRF);
                                //placePixels();
                                break;
                        }
                    } else {
                        switch(selectedAuto){
                            case "RF":
                                drive.followTrajectorySequence(rightRF);
                                //placePixels();
                        }
                    }
            }
        }
        //todo ask the super intelligent cats to fix the code

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