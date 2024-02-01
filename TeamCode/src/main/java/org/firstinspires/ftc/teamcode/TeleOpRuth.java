package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TeleOpRuth extends LinearOpMode {
    public DcMotorEx backleft;
    public DcMotorEx backright;
    public DcMotorEx frontleft;
    public DcMotorEx frontright;
    public DcMotorEx gate;
    public TouchSensor touch;
    public DcMotor arm;
    public Servo airplane;
    public Servo hooklatch;
    public DcMotor hook;

    public boolean hookInit() {
        hook = hardwareMap.get(DcMotor.class, "hook");
        hook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hook.setDirection(DcMotorSimple.Direction.FORWARD);
        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hooklatch = hardwareMap.get(Servo.class, "hooklatch");

        return hook != null && hooklatch != null;
    }

    public boolean arminit() {
        if (arm == null || touch == null) {
            arm = hardwareMap.get(DcMotor.class, "arm");
            touch = hardwareMap.get(TouchSensor.class, "touch");
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            arm.setPower(0.5);
            return false;
        } else if (touch.isPressed()) {
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return true;
        }
        return false;
    }

    public boolean gateinit() {
        gate = hardwareMap.get(DcMotorEx.class, "gate");
        gate.setDirection(DcMotorSimple.Direction.FORWARD);
        gate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gate.setTargetPosition(0);
        gate.setTargetPositionTolerance(5);
        gate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gate.setPower(0.5);

        return gate != null;
    }

    public boolean driveInit() {
        backleft = (DcMotorEx)(hardwareMap.get(DcMotor.class, "backleft"));
        backright = (DcMotorEx)(hardwareMap.get(DcMotor.class, "backright"));
        frontleft = (DcMotorEx)(hardwareMap.get(DcMotor.class, "frontleft"));
        frontright = (DcMotorEx)(hardwareMap.get(DcMotor.class, "frontright"));
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return backleft != null && backright != null && frontleft != null && frontright != null;
    }

    @Override
    public void runOpMode() {

        boolean drive = driveInit();
        boolean gate = gateinit();
        boolean hook = hookInit();
        double latch = hooklatch.getPosition();

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addLine()
                    .addData("DRIVE", drive)
                    .addData("ARM", arminit())
                    .addData("HOOK", hook)
                    .addData("GATE", gate);
            telemetry.update();
        }

        waitForStart();



    }

}
