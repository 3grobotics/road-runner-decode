package com.example.meepmeeptesting2;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.net.URL;
import java.nio.file.Paths;
import java.util.Objects;


public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                //Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, -21), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, -26), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-45, -26), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-46, -55), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-59, -55), Math.toRadians(90))
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(-69, -4), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-65, -55), Math.toRadians(90))
                .waitSeconds(0.25)














                /*.splineToLinearHeading(new Pose2d (-35, 38, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d (-47, 10, Math.toRadians(270)), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-47, 48), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-47,10), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-57,10), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-57,48), Math.toRadians (270))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-61,10), Math.toRadians (180))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-61,48), Math.toRadians (270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-39, 55, Math.toRadians(180)), Math.toRadians(0))

                .strafeTo(new Vector2d(-39,63.8))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(1, 40, Math.toRadians(359.9)), Math.toRadians(270))
                .strafeTo(new Vector2d(1,30))



               /* .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d (-9, 30, Math.toRadians(0)), Math.toRadians(270))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d (-39, 55, Math.toRadians(180)), Math.toRadians(90))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d (-39, 64, Math.toRadians(180)), Math.toRadians(90))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d (-8, 30, Math.toRadians(0)), Math.toRadians(270))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d (-39, 55, Math.toRadians(180)), Math.toRadians(90))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d (-39, 64, Math.toRadians(180)), Math.toRadians(90))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d (-7, 30, Math.toRadians(0)), Math.toRadians(270))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d (-39, 55, Math.toRadians(180)), Math.toRadians(90))

                /*.setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d (-39, 64, Math.toRadians(180)), Math.toRadians(90))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d (-6, 30, Math.toRadians(0)), Math.toRadians(270))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d (-39, 64, Math.toRadians(180)), Math.toRadians(90))





                .strafeTo(new Vector2d(-11, 30))
                .splineToLinearHeading(new Pose2d (-35, 38, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d (-47, 10, Math.toRadians(270)), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-47, 55), Math.toRadians(90))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48,55), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-59,10), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-59,48), Math.toRadians (270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-39, 55, Math.toRadians(180)), Math.toRadians(0))
                .strafeTo(new Vector2d(-39,63.8))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-4, 35, Math.toRadians(359.9)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-5, 30, Math.toRadians(359.9)), Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-39, 45, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-39,63.8))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(1, 34, Math.toRadians(0)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-39, 45, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-39,63.8))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(0, 30, Math.toRadians(0)), Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-59, 63, Math.toRadians(360)), Math.toRadians(180))
                 */








                .build());

        /* new push in
        .splineToLinearHeading(new Pose2d (-35, 38, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d (-47, 8, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-47,55, Math.toRadians(270)), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-59,10, Math.toRadians(270)), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-59,48, Math.toRadians(270)), Math.toRadians (90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, 37, Math.toRadians(180)), Math.toRadians(315))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-64, 24, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-64,55, Math.toRadians(180)), Math.toRadians(90))





        old push in
        .setTangent(90)
                .splineToLinearHeading(new Pose2d(-20, 45, Math.toRadians(-100)), Math.toRadians(135))
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(-31, 41, Math.toRadians(-100)), Math.toRadians(180))
                // arm stuff
                .splineToLinearHeading(new Pose2d(-34, 50, Math.toRadians(150)), Math.toRadians(90))
                //raise
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(-40, 41, Math.toRadians(-100)), Math.toRadians(180))
                // arm stuff
                .splineToLinearHeading(new Pose2d(-43, 50, Math.toRadians(150)), Math.toRadians(90))
                //raise
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(-50, 41, Math.toRadians(-100)), Math.toRadians(180))
                // arm stuff
                .splineToLinearHeading(new Pose2d(-53, 50, Math.toRadians(150)), Math.toRadians(90))*/
        URL url = Objects.requireNonNull(
                MeepMeepTesting2.class.getResource("/backgrounds/Juice-DECODE-Dark.png"),
                "Resource not found on classpath: /backgrounds/Juice-DECODE-Dark.png");


    meepMeep
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
}
}