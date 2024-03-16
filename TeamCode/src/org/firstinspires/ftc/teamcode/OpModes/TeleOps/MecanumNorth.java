package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name ="test bot demo", group = "MecanumNorth")
public class MecanumNorth extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor m1 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
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

        Servo back_servo = hardwareMap.servo.get("back_servo");

        back_servo.setPosition(0.5);
        back_servo.setPosition(0);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double tx = gamepad1.right_stick_x;
            double ty = gamepad1.right_stick_y;
            Orientation angle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            Orientation degree = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);





            // Start of Mecanum Drive
            double m = Math.atan2(y,x) - angle.firstAngle;
            double z = 3 * Math.sqrt(x*x + y*y);

            double p1 = Math.sin(m +  Math.PI/4) * z;
            double p2 = Math.sin(m -  Math.PI/4)* z;
            double p3 = Math.sin(m -  Math.PI/4) * z;
            double p4 = Math.sin(m +  Math.PI/4)* z;






            m1.setPower(p1+tx); //front left
            m2.setPower(p2-tx);//front right
            m3.setPower(p3+tx);//back left
            m4.setPower(p4-tx);//back right



            telemetry.addData("Left X Value", x);
            telemetry.addData("Left Y Value", y);
            telemetry.addData("Right X value", tx);
            telemetry.addData("Right Y Value", ty);
            telemetry.addData("Robot Radian", angle);
            telemetry.addData("Robot Degree", degree);
            telemetry.addData("Back Left", m1.getPower());
            telemetry.addData("Front Left", m2.getPower());
            telemetry.addData("Front Right", m3.getPower());
            telemetry.addData("Back Right", m4.getPower());
            telemetry.addData("Joystick Angle", m);
            telemetry.addData("Test", p1);
            telemetry.addData("Test", p2);
            telemetry.addData("Test", p3);
            telemetry.addData("Test", p4);
            telemetry.addData("Magnitude", z);
            telemetry.update();




        }
    }
}
