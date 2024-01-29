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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoBlueFar", group = ":3")
public class AutoBlueFar extends LinearOpMode {

    private Pose2d startPose = new Pose2d(-36.0, 60, Math.toRadians(-90.0));

    private Vector2d spikeVec = new Vector2d(startPose.getX(), startPose.getY()-23.0);

    private DcMotorEx arm;

    private final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    private TouchSensor touch;

    public void arminit() {
        arm = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "arm");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPositionPIDFCoefficients(4.0);

        arm.setPower(0.5);

        while (!touch.isPressed()) {
            telemetry.addData("RESETTING ARM", "");
            telemetry.update();
            sleep(20);
            idle();
        }

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void runOpMode() {



        drive.setPoseEstimate(startPose);

        arminit();

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

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
            .addSpatialMarker(new Vector2d(36.0, 60.0), () -> {
                //todo: start extending the arm here
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(1.0);
                arm.setTargetPosition(-24500);
            })
            .lineToSplineHeading(spikePose)
            .forward(7)
            .back(7)
            .lineToSplineHeading(new Pose2d(startPose.vec(), 0.0))
            .lineTo(new Vector2d(26.0, 60.0))
            .splineToConstantHeading(tagPose.vec(), tagPose.getHeading())
            .build();

        while (!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.update();

    }
}
