package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// Non-RR imports
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoBlueFar", group = ":3")
public class AutoBlueFar extends LinearOpMode {

    int s = 1;

    final private Pose2d startPose = new Pose2d(-36.0, 65 *s, Math.toRadians(-90.0 * s));

    private DcMotorEx arm;



    private SampleMecanumDrive drive;

    private TouchSensor touch;

    public void arminit() {

        arm = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "arm");
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
//    {
//
//        drive.setPoseEstimate(startPose);
//
//        arminit();
//
//        double angle = 20.0;
//
//        double heading = startPose.getHeading();
//
//        double tag = 0.0;
//
//        if (angle < -15.0) {
//            heading += Math.toRadians(60.0);
//            tag = 7.0;
//        } else if (angle > 15.0) {
//            //todo
//            heading -= Math.toRadians(60.0);
//            tag = -7.0;
//        }
//
//        Pose2d spikePose = new Pose2d(spikeVec, heading);
//
//        Pose2d tagPose = new Pose2d(48.0, 30.0+tag, 0.0);
//
//        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
//            .addSpatialMarker(new Vector2d(36.0, 60.0), () -> {
//                //todo: start extending the arm here
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                arm.setPower(1.0);
//                arm.setTargetPosition(-24500);
//            })
//            .lineToSplineHeading(spikePose)
//            .forward(7)
//            .back(7)
//            .lineToSplineHeading(new Pose2d(startPose.vec(), 0.0))
//
//            .splineToConstantHeading(tagPose.vec(), tagPose.getHeading())
//            .build();
//
//        while (!isStopRequested() && !opModeIsActive()) {
//

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        arminit();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(auto(0.0));


        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();
        }

    }

    public TrajectorySequence auto(double angle) {

        double tag = -7.5;

        if (!(angle >= -15.0 && angle <= 15.0)) {
            double heading = startPose.getHeading() - Math.copySign(Math.toRadians(30.0), angle);

            double x = startPose.getX() - Math.copySign(12.0, angle) + Math.copySign(9.0 * Math.cos(heading), angle);
            double y = Math.copySign(24.0, startPose.getY()) + Math.copySign(9.0 * Math.sin(heading), startPose.getY());

            if (angle < -15.0) {
                tag += 6.5;
            } else if (angle > 15.0) {
                tag += -6.5;
            }

            Pose2d spikePose = new Pose2d(x, y, heading);

            Pose2d tagPose = new Pose2d(59.0, Math.copySign(28.0, startPose.getY()) + tag, 0.0);

            return drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(4.0, () -> {
                    // todo: start extending the arm here
                    arm.setTargetPosition(-24500);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setPower(1.0);
                })
                .splineToSplineHeading(spikePose, spikePose.getHeading())
                // .forward(7)
                // .back(7)
                .splineToSplineHeading(new Pose2d(startPose.getX(), Math.copySign(48.0, startPose.getY()), startPose.getHeading()), Math.toRadians(90.0))
                .splineToConstantHeading(new Vector2d(-24, 60), 0.0)
                .lineTo(new Vector2d(26.0, Math.copySign(60.0, startPose.getY())))
                .splineToSplineHeading(tagPose, tagPose.getHeading())
                .build();
        } else {
            double heading = startPose.getHeading();

            double x = startPose.getX();
            double y = Math.copySign(31.0, startPose.getY());

            Pose2d spikePose = new Pose2d(x, y, heading);

            Pose2d tagPose = new Pose2d(59.0, Math.copySign(28.0, startPose.getY()) + tag, 0.0);
            return drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(4.0, () -> {
                        //todo: start extending the arm here
                        arm.setTargetPosition(-24500);
                        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        arm.setPower(1.0);
                    })
                    .splineToSplineHeading(spikePose, spikePose.getHeading())
                    //.forward(7)
                    //.back(7)
                    .back(5)
                    .lineTo(new Vector2d(startPose.getX(), Math.copySign(-48, startPose.getY())))
                    .splineToConstantHeading(new Vector2d(-24, 60), 0.0)
                    .lineTo(new Vector2d(26.0, Math.copySign(60.0, startPose.getY())))
                    .splineToSplineHeading(tagPose, tagPose.getHeading())
                    .waitSeconds(10.0)
                    .build();
        }
    }
}
