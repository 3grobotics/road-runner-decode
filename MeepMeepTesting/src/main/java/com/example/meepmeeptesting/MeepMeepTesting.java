package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.8)
                .setDimensions(10.8, 17)

                .build();

        VelConstraint adaptiveBrake = (robotPose, path, pathPos) -> {
            double distLeft = path.length() - pathPos;
            double cruiseVel = 45.0;
            double slowVel = 12.0;
            double brakeZone = 10.0;

            if (distLeft < brakeZone) {
                return slowVel + (cruiseVel - slowVel) * (distLeft / brakeZone);
            }
            return cruiseVel;
        };

                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-14.5, 15.5, Math.toRadians(145)))




                        .turnTo(Math.toRadians(90))
                        //.afterDisp(0, robot.intake())
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(37, 27, Math.toRadians(90)), Math.toRadians(90), adaptiveBrake)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(37, 53, Math.toRadians(90)), Math.toRadians(90), adaptiveBrake)

                        /*.setTangent(Math.toRadians(315))
                        .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(315), adaptiveBrake)
                        .waitSeconds(3)
                        
                        .waitSeconds(2)
                        

                        
                        .setTangent(Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(12, 50, Math.toRadians(90)), Math.toRadians(90), adaptiveBrake)


                        .setTangent(Math.toRadians(245))
                        .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(245), adaptiveBrake)
                        .waitSeconds(2)
                        
                        .waitSeconds(2)
                        

                        
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(4, 38, Math.toRadians(100)), Math.toRadians(90))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(4, 55, Math.toRadians(100)), Math.toRadians(90))

                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(20, 60, Math.toRadians(140)), Math.toRadians(45))


                        
                        
                        
































                        /*.setTangent(Math.toRadians(115))
                        .splineToLinearHeading(new Pose2d(60.5, 6.5 , Math.toRadians(150)), Math.toRadians(180), adaptiveBrake)
                        .waitSeconds(4)

                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(36, 54, Math.toRadians(90)), Math.toRadians(90))

                        .strafeToLinearHeading(new Vector2d(60.5, 6.5), Math.toRadians(150))
                        .waitSeconds(1)


                        //first gate pickup
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(46, 60, Math.toRadians(140)), Math.toRadians(90))
                        //intake
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(26, 60, Math.toRadians(140)), Math.toRadians(180))

                        .setTangent(Math.toRadians(315))
                        .splineToSplineHeading(new Pose2d(60.5, 6.5, Math.toRadians(150)), Math.toRadians(315))
                        .waitSeconds(1)


                        //second gate pickup
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(46, 60, Math.toRadians(140)), Math.toRadians(90))
                        //intake
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(26, 60, Math.toRadians(140)), Math.toRadians(180))

                        .setTangent(Math.toRadians(315))
                        .splineToSplineHeading(new Pose2d(60.5, 6.5, Math.toRadians(150)), Math.toRadians(315))
                        .waitSeconds(1)


                        //third gate pickup
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(46, 60, Math.toRadians(140)), Math.toRadians(90))
                        //intake
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(26, 60, Math.toRadians(140)), Math.toRadians(180))

                        .setTangent(Math.toRadians(315))
                        .splineToSplineHeading(new Pose2d(60.5, 6.5, Math.toRadians(150)), Math.toRadians(315))
                        .waitSeconds(1)*/



                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}