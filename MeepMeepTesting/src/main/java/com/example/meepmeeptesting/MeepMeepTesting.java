package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    static Pose2d startPose = new Pose2d(-36.0, 60, Math.toRadians(-90.0));

    static Vector2d spikeVec = new Vector2d(startPose.getX(), startPose.getY()-23.0);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double angle = 20.0;

        double heading = startPose.getHeading();

        double tag = 0.0;

        if (angle < -15.0) {
            heading += Math.toRadians(60.0);
            tag = 7.0;
        } else if (angle > 15.0) {
            //todo
            heading -= Math.toRadians(60.0);
            tag = -7.0;
        }

        Pose2d spikePose = new Pose2d(spikeVec, heading);

        Pose2d tagPose = new Pose2d(48.0, 30.0+tag, 0.0);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(41, 41, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .addSpatialMarker(new Vector2d(36.0, 60.0), () -> {
                                    //todo: start extending the arm here
                                })
                                .lineToSplineHeading(spikePose)
                                .forward(7)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(startPose.vec(), 0.0))
                                .lineTo(new Vector2d(26.0, 60.0))
                                .splineToConstantHeading(tagPose.vec(), tagPose.getHeading())
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
