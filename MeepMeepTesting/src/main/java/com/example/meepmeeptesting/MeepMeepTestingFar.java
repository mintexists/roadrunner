package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingFar {


    static int s = -1;
    static Pose2d startPose = new Pose2d(-36.0, 65*s, Math.toRadians(-90.0 * s));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double angle = 0.0;
        int a = (angle < 0 ? -1 : 1);
        RoadRunnerBotEntity myBot;

        double tag = -6.5;

        if (!(angle >= -15.0 && angle <= 15.0)) {
            double heading = startPose.getHeading() - Math.toRadians(50.0) * a;
//              * s
            double x = startPose.getX() - 13.0 * a * s - 7.0 * Math.cos(heading);
//             + s * a *
            double y = 32.5 * s - 7.0 * Math.sin(heading);


            if (angle < -15.0) {
                tag += 6.5;
            } else if (angle > 15.0) {
                tag += -6.5;
            }

            Pose2d spikePose = new Pose2d(x, y, heading);

            Pose2d tagPose = new Pose2d(45.0, 36.0 * s + tag, 0.0);


            myBot = new DefaultBotBuilder(meepMeep)
                    .setDimensions(17.5, 17.5)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(41, 41, Math.toRadians(180), Math.toRadians(180), 17.5)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(startPose)
                                            .addTemporalMarker(3.0, () -> {
//                                                arm.setTargetPosition(-24500);
//                                                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                                                arm.setPower(1.0);
                                            })
                                            .forward(13.0)
                                            .splineToSplineHeading(spikePose, spikePose.getHeading())
                                            .setReversed(true)
                                            .splineToSplineHeading(new Pose2d(startPose.getX(), startPose.getY() - (13 * s), startPose.getHeading()), Math.toRadians(90.0 * s))
                                            .setReversed(false)
                                            .lineTo(new Pose2d(startPose.getX(), 60.0 * s, startPose.getHeading()).vec())
                                            .turn(Math.toRadians(90.0 * s))
                                            .lineTo(new Vector2d(24.0, 59.0 * s))
                                            .splineToLinearHeading(tagPose, Math.toRadians(-90.0 * s))
                                            .waitSeconds(2)
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
                                    .addSpatialMarker(new Vector2d(24.0, 59.0*s), () -> {
                                        //todo: start extending the arm here
                                    })
                                    .forward(13.0)
                                    .splineToSplineHeading(spikePose, spikePose.getHeading())
                                    .lineTo(new Pose2d(startPose.getX(), 60.0 * s, startPose.getHeading()).vec())
                                    .turn(Math.toRadians(90.0 * s))
                                    .lineTo(new Vector2d(24.0, 59.0 * s))
                                    .splineToLinearHeading(tagPose, Math.toRadians(-90.0 * s))
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