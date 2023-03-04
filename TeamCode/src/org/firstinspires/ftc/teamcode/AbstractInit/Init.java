package org.firstinspires.ftc.teamcode.AbstractInit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Drivers.Drivers;
import org.firstinspires.ftc.teamcode.Drivers.Mecanum;
import org.firstinspires.ftc.teamcode.Extra.disabled_samples.EncBot;
import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.Hardware.Commands;
import org.firstinspires.ftc.teamcode.Hardware.MotorData;
import org.firstinspires.ftc.teamcode.Helpers.BulkReader;
import org.firstinspires.ftc.teamcode.NewSelfDriving.AutoDrive;
import org.firstinspires.ftc.teamcode.OpModes.OpScript;
import org.firstinspires.ftc.teamcode.SelfDriving.PID;
import org.firstinspires.ftc.teamcode.SelfDriving.SelfDriving;
import org.firstinspires.ftc.teamcode.SelfDriving.SelfTurning;
import org.openftc.easyopencv.OpenCvCameraFactory;

/**
 * Inheritance class for Bot
 * Contains objects, init for Bot construcotr, and select opMode capabilities
 */
public abstract class Init {
    protected static OpScript opmode;
    public static IMU imuC;
    public static long cycleNumber = 0;
    public static Telemetry telemetry;
    public static int barcode = -1;
    public static boolean runAuto;
    public static BulkReader bulkReader;
    public static PID pid;
    public static MotorData motorData;
    public static Drivers drivers;
    public static Mecanum mecanum;
    public static SelfDriving selfDriving;
    public static Commands commands;
    public static SelfTurning selfTurning;
    public static AutoDrive autoDrive;

    /**
     *
     * @param opMode: used to initalize Bot constructor
     */
    protected void init(OpScript opMode) {
        opmode = opMode;
        barcode = opMode.barcode;
        runAuto = opMode.runAuto;
        telemetry = opMode.telemetry;
        opMode.bulkReader = new BulkReader(opMode.hardwareMap);
        bulkReader = opMode.bulkReader;
        imuC = opMode.hardwareMap.get(IMU.class, "imu");
        //instantiating the previously null objects
        imuC.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imuC.resetYaw();
        Motors.setMotors(opMode.hardwareMap.get(DcMotorEx.class, Config.motorLF),
                opMode.hardwareMap.get(DcMotorEx.class, Config.motorRF),
                opMode.hardwareMap.get(DcMotorEx.class, Config.motorLB),
                opMode.hardwareMap.get(DcMotorEx.class, Config.motorRB),
                opMode.hardwareMap.get(DcMotorEx.class, Config.odoLeft),
                opMode.hardwareMap.get(DcMotorEx.class, Config.odoMiddle),
                opMode.hardwareMap.get(DcMotorEx.class, Config.odoRight));
        motorData = new MotorData();
        mecanum = new Mecanum();
        drivers = new Drivers();
        commands = new Commands();
        pid = new PID();
        autoDrive = new AutoDrive();
        selfDriving = new SelfDriving();
        selfTurning = new SelfTurning();
    }

    public static class Motors {
        public static DcMotorEx leftFront;
        public static DcMotorEx rightFront;
        public static DcMotorEx leftBack;
        public static DcMotorEx rightBack;
        public static DcMotorEx odoLeft;
        public static DcMotorEx odoMiddle;
        public static DcMotorEx odoRight;

        public static void setMotors(DcMotorEx lFront, DcMotorEx rFront, DcMotorEx lBack, DcMotorEx rBack, DcMotorEx odoleft, DcMotorEx odomiddle, DcMotorEx odoright) {
            leftFront = lFront;
            rightFront = rFront;
            leftBack = lBack;
            rightBack = rBack;
            odoLeft = odoleft;
            odoRight = odoright;
            odoMiddle = odomiddle;
        }
    }
}
