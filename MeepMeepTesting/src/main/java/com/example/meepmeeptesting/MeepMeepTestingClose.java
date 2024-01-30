package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingClose {



    static int s = 1;
    static Pose2d startPose = new Pose2d(12.0, 60*s, Math.toRadians(-90.0 * s));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        double angle = 20.0;
        int a = (angle < 0 ? -1 : 1);
        RoadRunnerBotEntity myBot;

        double tag = -7.5;

        if (!(angle >= -15.0 && angle <= 15.0)) {
//            double heading = startPose.getHeading() - Math.toRadians(30.0) * a;
//
//            double x = startPose.getX() + 12.0 * a * s - 9.0 * Math.cos(heading);
//            double y = 24.0 * s - 9.0 * Math.sin(heading);
            double heading = startPose.getHeading() - Math.toRadians(30.0)*a;
//              * s
            double x = startPose.getX() - 12.0 * a * s - + 9.0 * Math.cos(heading);
//             + s * a *
            double y = 24.0 * s - 9.0 * Math.sin(heading);


            if (angle < -15.0) {
                tag += 6.5;
            } else if (angle > 15.0) {
                tag += -6.5;
            }

            Pose2d spikePose = new Pose2d(x, y, heading);

            Pose2d tagPose = new Pose2d(48.0, 36.0 * s + tag, 0.0);


            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(41, 41, Math.toRadians(180), Math.toRadians(180), 17.5)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(startPose)
                                            .addSpatialMarker(new Vector2d(36.0, 60.0), () -> {
                                                //todo: start extending the arm here
                                            })
                                            .splineToSplineHeading(spikePose, spikePose.getHeading())
//                                .forward(7)
//                                .back(7)
                                            .lineToSplineHeading(new Pose2d(startPose.getX(), 48.0 * s, startPose.getHeading()))
//                                            .splineToSplineHeading(new Pose2d(startPose.getX()+5, Math.copySign(48.0, startPose.getY()), startPose.getHeading()), Math.toRadians(90.0))
                                            .splineToConstantHeading(new Vector2d(24, startPose.getY()), 0.0)
//                                            .splineToSplineHeading(new Pose2d(24, startPose.getY(), startPose.getHeading()), 0.0)
                                            .splineToSplineHeading(tagPose, Math.toRadians(0.0))
                                            .forward(2)
                                            .build()
                    );
        } else {
            double heading = startPose.getHeading();

            double x = startPose.getX();
            double y = 29.0 * s;

            Pose2d spikePose = new Pose2d(x, y, heading);

            Pose2d tagPose = new Pose2d(48.0, 36.0 * s + tag, 0.0);
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(41, 41, Math.toRadians(180), Math.toRadians(180), 16)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(startPose)
                                            .addSpatialMarker(new Vector2d(36.0, 60.0), () -> {
                                                //todo: start extending the arm here
                                            })
                                            .splineToSplineHeading(spikePose, spikePose.getHeading())
//                                .forward(7)
//                                .back(7)
                                            .lineToSplineHeading(new Pose2d(startPose.getX(), 48.0 * s, startPose.getHeading()))
//                                            .splineToSplineHeading(new Pose2d(startPose.getX()+5, Math.copySign(48.0, startPose.getY()), startPose.getHeading()), Math.toRadians(90.0))
                                            .splineToConstantHeading(new Vector2d(24, startPose.getY()), 0.0)
//                                            .splineToSplineHeading(new Pose2d(24, startPose.getY(), startPose.getHeading()), 0.0)
                                            .lineTo(new Vector2d(24.0, startPose.getY()))
                                            .splineToSplineHeading(tagPose, Math.toRadians(0.0))
                                            .forward(2)
                                            .build()
                    );
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}