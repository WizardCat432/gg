package com.example.meepmeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.567, 52.567, Math.toRadians(223.1014), Math.toRadians(223.1014), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, 63.50, Math.toRadians(-85.00)))
                                .splineToLinearHeading(new Pose2d(-35.00, 35.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                                .lineTo(new Vector2d(-30,35))
                                .lineTo(new Vector2d(-37,35))
                                .lineTo(new Vector2d(-40, 35))
                                .lineTo(new Vector2d(-40.00, 33.00))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(-37.00, 12.00))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(38.00, 12.00))
                                .lineTo(new Vector2d(60, 25))
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

//270

