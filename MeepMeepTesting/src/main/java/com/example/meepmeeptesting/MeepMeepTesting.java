package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    static Pose2d startPose = new Pose2d(-36.0, 65, Math.toRadians(-90.0));

    static Vector2d spikeVec = new Vector2d(startPose.getX(), startPose.getY()-23.0);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double angle = -15.0;

        double tag = -7.5;

        double heading = startPose.getHeading() - Math.copySign(Math.toRadians(30.0), angle);

        double x = startPose.getX() - Math.copySign(12.0, angle) + Math.copySign(9.0 * Math.cos(heading), angle);
        double y = Math.copySign(24.0, startPose.getY()) + Math.copySign(9.0 * Math.sin(heading), startPose.getY());


        if (angle < -15.0) {
            tag += 6.5;
        } else if (angle > 15.0) {
            tag += -6.5;
        }
        Pose2d spikePose = new Pose2d(x, y, heading);

        Pose2d tagPose = new Pose2d(59.0, Math.copySign(36.0, startPose.getY()) + tag, 0.0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(41, 41, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .addSpatialMarker(new Vector2d(36.0, 60.0), () -> {
                                    //todo: start extending the arm here
                                })
                                .splineToSplineHeading(spikePose, spikePose.getHeading())
                                // .forward(7)
                                // .back(7)
                                //worried about the last radians thing in this line, need to check it
                                .splineToSplineHeading(new Pose2d(startPose.getX(), Math.copySign(48.0, startPose.getY()), startPose.getHeading()), Math.toRadians(90.0))
                                .splineToConstantHeading(new Vector2d(-24, 60), 0.0)
                                .lineTo(new Vector2d(26.0, Math.copySign(60.0, startPose.getY())))
                                .splineToSplineHeading(tagPose, tagPose.getHeading())
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
