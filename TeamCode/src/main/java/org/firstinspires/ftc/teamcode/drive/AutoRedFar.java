package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoRedFar", group = ":3")
public class AutoRedFar extends LinearOpMode {

    static private final int s = -1;

    static private final Pose2d startPose = new Pose2d(-36.0, 65 * s, Math.toRadians(-90.0 * s));

    private static DcMotorEx arm;

    static private SampleMecanumDrive drive;

    private TouchSensor touch;

    //return drive.trajectorySequenceBuilder(startPose)
    public static TrajectorySequence auto(double angle, SampleMecanumDrive drive, DcMotor arm) {
        int a = (angle < 0 ? -1 : 1);

        double tag = -6.5;

        if (!(angle >= -15.0 && angle <= 15.0)) {
            double heading = startPose.getHeading() - Math.toRadians(50.0) * a;
//              * s
            double x = startPose.getX() - 13.0 * a * s - 7.0 * Math.cos(heading);
//             + s * a *
            double y = 32.5 * s - 7.0 * Math.sin(heading);


            if (angle < -15.0) {
                tag += 6.0;
            } else if (angle > 15.0) {
                tag += -6.0;
            }

            Pose2d spikePose = new Pose2d(x, y, heading);

            Pose2d tagPose = new Pose2d(49.0, 36.0 * s + tag, 0.0);

            return drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(3.0, () -> {
                        arm.setTargetPosition(-24500);
                        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        arm.setPower(1.0);
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
//                    .waitSeconds(2)
                    .build();
        } else {
            double heading = startPose.getHeading();

            double x = startPose.getX();
            double y = 29.0 * s;

            Pose2d spikePose = new Pose2d(x, y, heading);

            Pose2d tagPose = new Pose2d(49.0, 36.0 * s + tag, 0.0);
            return drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(3.0, () -> {
                        arm.setTargetPosition(-24500);
                        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        arm.setPower(1.0);
                    })
                    .forward(13.0)
                    .splineToSplineHeading(spikePose, spikePose.getHeading())
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(startPose.getX(), 47.0 * s, startPose.getHeading()), Math.toRadians(90.0 * s))
                    .setReversed(false)
                    .lineTo(new Pose2d(startPose.getX(), 60.0 * s, startPose.getHeading()).vec())
                    .turn(Math.toRadians(90.0 * s))
                    .lineTo(new Vector2d(24.0, 59.0 * s))
                    .splineToLinearHeading(tagPose, Math.toRadians(-90.0 * s))
//                    .waitSeconds(1.0)
                    .build();
        }
    }

    public void arminit() {

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPositionPIDFCoefficients(4.0);

        arm.setPower(0.5);

        while (!touch.isPressed()) {
            telemetry.addData("RESETTING ARM", "");
            telemetry.update();
            sleep(20);
            idle();
        }

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        arminit();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(auto(0.0, drive, arm));

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();
        }

    }
}
