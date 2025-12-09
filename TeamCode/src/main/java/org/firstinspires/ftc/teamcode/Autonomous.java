package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", preselectTeleOp = "FCDrivingITD")
public class Autonomous extends LinearOpMode {
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private DcMotor RightIntake;
    GoBildaPinpointDriver Odometry; // Declare OpMode member for the Odometry Computer
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
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //port 1
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //port 2
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //port 3
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightIntake = hardwareMap.get(DcMotor.class, "RightIntake");
        RightIntake.setDirection(DcMotor.Direction.REVERSE);

        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Odometry.setOffsets(-30, 0);
        Odometry.recalibrateIMU();
        Odometry.resetPosAndIMU();
        Odometry.recalibrateIMU();
        Odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //Odometry.setPosition(Pose2D.from);

        waitForStart();

        Drive_Controls(-0, 0, 200, 15);


        waitForStart();

        Drive_Controls(-0, 0, 200, 15);
    }
    private void Drive_Controls(int TargetHeading, int TargetX, int TargetY, int Tolerance) {
        boolean InXPosition = Math.abs(TargetX - Odometry.getPosX()) <= Tolerance;
        boolean InYPosition = Math.abs(TargetY - Odometry.getPosY()) <= Tolerance;
        boolean correctPosition = InXPosition && InYPosition;

        while (!correctPosition) {
            //Inspired by https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html and used microsoft copilot to help refine code

            Odometry.update();

            double y = (TargetY - Odometry.getPosX())/25;
            double x = (TargetX - Odometry.getPosY())/25;

            double botHeading = Odometry.getHeading();
            telemetry.addData("Yaw: ", botHeading);

            double radTargetHeading = Math.toRadians(TargetHeading);
            double rx = radTargetHeading - botHeading;

            telemetry.addData("rx: ", rx);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double FLDrivePower = (rotY - rotX + rx) / denominator;
            double BLDrivePower = (rotY - rotX - rx) / denominator;
            double FRDrivePower = (rotY + rotX - rx) / denominator;
            double BRDrivePower = (rotY + rotX + rx) / denominator;

            FLDrive.setPower(FLDrivePower);
            BLDrive.setPower(BLDrivePower);
            FRDrive.setPower(FRDrivePower);
            BRDrive.setPower(BRDrivePower);

            telemetry.addData("FLDrive ", FLDrivePower);
            telemetry.addData("BLDrive ", BLDrivePower);
            telemetry.addData("FRDrive ", FRDrivePower);
            telemetry.addData("BRDrive ", BRDrivePower);

            InXPosition = Math.abs(TargetX - Odometry.getPosX()) <= Tolerance;
            InYPosition = Math.abs(TargetY - Odometry.getPosY()) <= Tolerance;

            correctPosition = InXPosition && InYPosition;

            telemetry.addData("PosX: ", Odometry.getPosX());
            telemetry.addData("PosY: ", Odometry.getPosY());

            telemetry.update();
        }
    }
}

