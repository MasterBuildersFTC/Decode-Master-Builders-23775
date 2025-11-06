package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "FCDrivingITD", group = "LinearOpMode")
public class Field_Centric_TeleOp extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime();
    GoBildaPinpointDriver Odometry; // Declare OpMode member for the Odometry Computer
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    public void runOpMode() {
        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");

        //Making Sure wheels are turning in the right direction
        //port 0
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Note: To drive, the robot requires one motor to be turning in an opposite direction
        //port 1
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //port 2
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //port 3
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runTime.reset();

        //REMOVE WHEN AUTONOMOUS IS IN PLACE
        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Odometry.setOffsets(-68.0, 0);
        Odometry.recalibrateIMU();
        Odometry.resetPosAndIMU();
        Odometry.recalibrateIMU();
        Odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        while (opModeIsActive()) {
            Drive_Controls();
            telemetry.update();
        }
    }
    //General OpMode Specific Functions
    private void Drive_Controls() {

        //Inspired by https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html and used microsoft copilot to help refine code

        Odometry.update();
        double y = ((gamepad1.left_stick_y) + (gamepad1.right_stick_y * .25));
        double x = -((gamepad1.left_stick_x) + (gamepad1.right_stick_x * .25));
        double rx = ((gamepad1.left_trigger - gamepad1.right_trigger) + ((gamepad2.left_trigger - gamepad2.right_trigger) * 0.25));
        double botHeading = Odometry.getHeading();
        telemetry.addData("Yaw: ", botHeading);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double FLDrivePower = (rotY + rotX + rx) / denominator;
        double BLDrivePower = (rotY - rotX + rx) / denominator;
        double FRDrivePower = (rotY + rotX - rx) / denominator;
        double BRDrivePower = (rotY - rotX - rx) / denominator;


        FLDrive.setPower(FLDrivePower);
        BLDrive.setPower(BLDrivePower);
        FRDrive.setPower(FRDrivePower);
        BRDrive.setPower(BRDrivePower);

        telemetry.addData("FLDrive ", FLDrivePower);
        telemetry.addData("BLDrive ", BLDrivePower);
        telemetry.addData("FRDrive ", FRDrivePower);
        telemetry.addData("BRDrive ", BRDrivePower);
    }
}
