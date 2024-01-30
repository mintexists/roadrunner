package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "PID TESTS")
public class PIDdriver extends LinearOpMode {

    final double MOTOR_MAX_RPM = 6000.0;

    final double GEAR = 18.88;

    final double MAX_RPM = MOTOR_MAX_RPM/GEAR;

    final double TICKS_PER_REV = 28.0;

    final double MAX_TICKS_PER_SEC = MAX_RPM * TICKS_PER_REV * GEAR / 60.0;


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
