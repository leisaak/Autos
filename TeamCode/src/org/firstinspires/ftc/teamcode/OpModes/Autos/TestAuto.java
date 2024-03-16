package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Features.Config;

@TeleOp (name = "TestAuto", group = "Autos")
public class TestAuto extends LinearOpMode {

    DcMotorEx m1, m2, m3, m4, odoLeft, odoRight, odoHang;
    Servo back_servo;

    @Override
    public void runOpMode() {
        m1 = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        m2 = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        m3 = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        m4 = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        odoLeft = hardwareMap.get(DcMotorEx.class, "enc_left");
        odoRight = hardwareMap.get(DcMotorEx.class, "enc_right");
        odoHang = hardwareMap.get(DcMotorEx.class, "enc_x");


        m1.setDirection(DcMotorEx.Direction.REVERSE);
        m3.setDirection(DcMotorEx.Direction.REVERSE);
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        back_servo = hardwareMap.get(Servo.class, "back_servo");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);

        ElapsedTime length = new ElapsedTime();
        int odoTicksPerRotation = 8192;
        double odoDiameterIn = 2;
        double odoDiameterCM = odoDiameterIn * 2.54;
        double odoRadiusCM = 2.54;
        double odoCircumferenceCM = odoDiameterCM * Math.PI;
        double odoTicksToCm = odoCircumferenceCM / odoTicksPerRotation;


        telemetry.update();

        waitForStart();
        length.reset();
        while (opModeIsActive()) {
            double distance = odoLeft.getCurrentPosition() * odoTicksToCm;

            boolean forward = true;


            if (distance > -10) {
                m1.setPower(0.5);
                m2.setPower(0.5);
                m3.setPower(0.5);
                m4.setPower(0.5);
            } else if (distance < -10) {
                forward = false;
            } if (forward == false && distance > -20) {
                m1.setPower(-0.5);
                m2.setPower(-0.5);
                m3.setPower(-0.5);
                m4.setPower(-0.5);
            }
            else {
                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(0);
            }
// Driving using seconds.
//            if (length.seconds() < 5){
//                m1.setPower(0.5);
//                m2.setPower(0.5);
//                m3.setPower(0.5);
//                m4.setPower(0.5);
//            }
//            else{
//                m1.setPower(0);
//                m2.setPower(0);
//                m3.setPower(0);
//                m4.setPower(0);
//            }

                telemetry.addLine("Hulloa");
                telemetry.addData("Timer", length);
                telemetry.addData("Time", time);
                telemetry.addData("Angle", imu.getAngularOrientation().firstAngle);
                telemetry.addData("Ticks", odoLeft.getCurrentPosition());
                telemetry.addData("OdoLeft", distance);
                telemetry.addData("Forward?", forward);
                telemetry.update();
            }


        }
    }

