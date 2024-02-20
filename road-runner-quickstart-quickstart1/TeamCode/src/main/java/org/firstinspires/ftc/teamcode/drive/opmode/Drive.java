package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether tLocalizationTesthe localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Drive")
public class Drive extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor intakeMotor = null;

    //private DcMotor liftMotor = null;

    private DcMotor slideMotor;

    private DcMotor firstLift = null;
    private DcMotor secondLift = null;

    private CRServo gripServo = null;

    private Servo positionServo = null;

    private Servo planeServo = null;

    private AnalogInput potentiometer;
    private double currentVoltage;




    @Override
    public void runOpMode() throws InterruptedException {
        timer.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        potentiometer =hardwareMap.get(AnalogInput.class, "potentiometer");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        //liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        firstLift = hardwareMap.get(DcMotor.class, "firstLift");
        secondLift = hardwareMap.get(DcMotor.class, "secondLift");

        gripServo = hardwareMap.get(CRServo.class, "gripServo");
        positionServo = hardwareMap.get(Servo.class, "positionServo");
        planeServo = hardwareMap.get(Servo.class, "planeServo");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        planeServo.setPosition(0);

        //drive.setPoseEstimate(Storage.currentPose);
        drive.setPoseEstimate(new Pose2d());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            Pose2d poseEstimate = drive.getPoseEstimate();

//            Vector2d input = new Vector2d(
//                    -gamepad1.left_stick_x,
//                    -gamepad1.left_stick_y
//            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                             -gamepad1.right_stick_x
                    )
            );

            drive.update();

            // TODO teach evil cats to take over the world

            //Slide lift code
            double slidePosition = slideMotor.getCurrentPosition();
            if (slidePosition > -100 && slidePosition < 5000) {
                slideMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            } else if (slidePosition < -100) {
                slideMotor.setPower(1);
            } else if (slidePosition > 5000) {
                slideMotor.setPower(-1);
            }
            telemetry.addData("slidePosition", slidePosition);

            //TODO Use this one! (ELI! LOOK DOWN!)
            currentVoltage = potentiometer.getVoltage();
            if(slidePosition > 3500 && currentVoltage > 0.8){
                gripServo.setPower(-1);
            } else if(slidePosition < 3500 && currentVoltage < 1.9) {
                gripServo.setPower(1);
            }else if(currentVoltage < 0.8){
                gripServo.setPower(0);
            }else if (currentVoltage > 1.9){
                gripServo.setPower(0);
            }


            //servos for intake
            if (gamepad1.right_bumper) {
                positionServo.setPosition(1);
                telemetry.addData("positionPosition", positionServo.getPosition());
            } else if (gamepad1.left_bumper) {
                positionServo.setPosition(0.65);
                telemetry.addData("positionPosition", positionServo.getPosition());
            }

            //intake Code
            if (gamepad1.b) {
                intakeMotor.setPower(1);
            } else if (gamepad1.a) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            //TODO teach the cats how to use lasers for destruction

            if (gamepad2.x) {
                firstLift.setPower(-1);
                secondLift.setPower(1);
            } else if (gamepad2.y) {
                firstLift.setPower(1);
                secondLift.setPower(-1);
            } else {
                firstLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                secondLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               firstLift.setPower(0);
               secondLift.setPower(0);
            }

            // Plane Servo
            if (gamepad2.a){
                planeServo.setPosition(1);
            } else {
                planeServo.setPosition(0.7);
            }

            //TODO Fix the laser shark tank as the cats misinterpreted

            telemetry.addData("timer", timer.time());
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("intake", intakeMotor.getPower());
                telemetry.addData("plain servo pos:", planeServo.getPosition());
                telemetry.addData("slidePosition", slidePosition);
                telemetry.addData("Potentiometer", currentVoltage);
                telemetry.addData("Left Odo", drive.leftFront.getCurrentPosition());
                telemetry.addData("Right Odo", drive.rightFront.getCurrentPosition());
                telemetry.addData("Front Odo", drive.leftRear.getCurrentPosition());
                telemetry.addData("FirstLift", firstLift.getCurrentPosition());
                telemetry.addData("SecondLift", secondLift.getCurrentPosition());
                telemetry.addData("SecLiftZeroPower:", secondLift.getZeroPowerBehavior());
                telemetry.addData("FirstLiftZeroPower:", firstLift.getZeroPowerBehavior());
                telemetry.addData("SlideCount:", slideMotor.getCurrentPosition());
                telemetry.addData("Right Trigger", gamepad1.right_trigger);
                telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.update();
            }
        }
    }
