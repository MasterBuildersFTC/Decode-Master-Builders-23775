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

        Drive_Controls(0, 0, 400, 40, 30);
        Drive_Controls(0,200,0,30,30);


    }
    private void Drive_Controls(int TargetHeading, int TargetX, int TargetY, int Tolerance, int SpeedControl) {
        PID distanceController = new PID();
        PID angleController = new PID();  // make sure this follows "Dealing with Angles"

        boolean inPostion = false;

        while (!inPostion) {
            Odometry.update();
            double robotTheta = Odometry.getHeading();
            double robotX = Odometry.getPosX();
            double robotY = Odometry.getPosY();

            double xError = TargetX - robotX;
            double yError = TargetY - robotY;
            double theta = Math.atan2(yError,xError);
            // 0 is the reference because we want the distance to go to 0
            double distance = Math.hypot(xError, yError);
            double left_power = f + t;
            double right_power = f - t;
            if (distance < threshold) {
                f = 0;
                t = angleController.calculate(targetAngle, robotTheta);
            } else {
                f = distanceController.calculate(0, distance);
                t = angleController.calculate(theta, robotTheta);
            }
            // Range.clip is included in the SDK and will clip between two values
            // angleController.error is a demonstrative attribute that gets the error.
            f *= Math.cos(Range.clip(angleController.error, -PI/2, PI/2));

            // set motor power here!
            FLDrive.setPower(f + t);
            BLDrive.setPower(f + t);
            FRDrive.setPower(f - t);
            BRDrive.setPower(f - t);

            telemetry.update();
        }
    }
}

