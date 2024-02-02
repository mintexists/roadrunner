package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpRuth")
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
    public DcMotorEx hook;

    public boolean hookInit() {
        hook = hardwareMap.get(DcMotorEx.class, "hook");
        hook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hook.setDirection(DcMotorSimple.Direction.FORWARD);
        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hooklatch = hardwareMap.get(Servo.class, "hooklatch");
        hooklatch.setPosition(0);

        return hook != null && hooklatch != null;
    }

    public boolean airplaineInit() {
        airplane = hardwareMap.get(Servo.class, "airplane");
        airplane.setPosition(0);

        return airplane != null;
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
        gate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        boolean gateinit = gateinit();
        boolean hookinit = hookInit();
        double latch = hooklatch.getPosition();
        boolean apinit = airplaineInit();
        double launch = airplane.getPosition();

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addLine()
                    .addData("DRIVE", drive)
                    .addData("ARM", arminit())
                    .addData("HOOK", hookinit)
                    .addData("AIRPLANE", apinit)
                    .addData("GATE", gateinit);
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                double boost = Math.abs(3.0 - gamepad1.right_trigger * 2.0);

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double power = Math.hypot(x, y);
                double theta = Math.atan2(y, x);

                double sin = Math.sin(theta - Math.PI/4);
                double cos = Math.cos(theta - Math.PI/4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                double bl = (power * sin/max);// / boost;
                double br = (power * cos/max);// / boost;
                double fl = (power * cos/max);// / boost;
                double fr = (power * sin/max);// / boost;

                backleft.setPower(bl + turn);
                backright.setPower(fl - turn);
                frontleft.setPower(br + turn);
                frontright.setPower(fr - turn);

                airplane.setPosition(gamepad2.a ? 1 : 0);

                if (gamepad2.b && hooklatch.getPosition() == latch) {
                    hooklatch.setPosition(1 - hooklatch.getPosition());
                }

                hook.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

                double gatep = gamepad1.right_bumper ? 0.25 : (gamepad1.left_bumper ? -0.25 : 0);

                gate.setPower(gatep);

            }
        }

    }

}
