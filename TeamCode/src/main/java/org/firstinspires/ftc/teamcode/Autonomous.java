package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", preselectTeleOp = "FCDrivingITD")
public class Autonomous extends LinearOpMode {

    private static final double kPx = 0.056, kIx = 0.0, kDx = 0.0059;
    private static final double kPy = 0.057, kIy = 0.0, kDy = 0.007;
    private static final double kPt = 0.02, kIt = 0.0, kDt = 0.005;

    private final PIDController xController = new PIDController(kPx, kIx, kDx);
    private final PIDController yController = new PIDController(kPy, kIy, kDy);
    private final PIDController thetaController = new PIDController(kPt, kIt, kDt);
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    //private DcMotor FrontIntake;
    //private DcMotor BackIntake;
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
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //port 1
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //port 2
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //port 3
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //FrontIntake = hardwareMap.get(DcMotor.class, "FrontIntake");
        //BackIntake = hardwareMap.get(DcMotor.class, "BackIntake");
        //FrontIntake.setDirection(DcMotor.Direction.REVERSE);
        //BackIntake.setDirection(DcMotor.Direction.REVERSE);

        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Odometry.setOffsets(-30, 0);
        Odometry.recalibrateIMU();
        Odometry.resetPosAndIMU();
        Odometry.recalibrateIMU();
        Odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //Odometry.setPosition(Pose2D.from);

        waitForStart();

        Drive_Controls(200,0,0,50,5,300000);

        //Drive_Controls(300,0,0,50,5,300000);

    }

    private void Drive_Controls(double targetX, double targetY, double targetAngle, double posTolerance, double angleTolerance, long timeoutMillis) {

        long start = System.currentTimeMillis();

        // Set PID setpoints
        xController.setSetPoint(targetX);
        yController.setSetPoint(targetY);

        angleTolerance = Math.toRadians(angleTolerance);
        targetAngle = Math.toRadians(targetAngle);

        while (opModeIsActive()) {
            Odometry.update();
            // Timeout safety
            if (System.currentTimeMillis() - start > timeoutMillis) break;

            // Read odometry (replace with your actual odometry calls!)
            double robotX     = Odometry.getPosX();
            double robotY     = Odometry.getPosY();
            double robotTheta = Odometry.getHeading(); // radians

            telemetry.addData("robotX: "   , robotX);
            telemetry.addData("robotY: "   , robotY);
            telemetry.addData("robotTheta:", robotTheta);

            // PID outputs for X/Y
            double cmdX_field = xController.calculate(robotX);
            double cmdY_field = yController.calculate(robotY);

            // Rotate into robot frame
            double cos = Math.cos(robotTheta);
            double sin = Math.sin(robotTheta);
            double cmdX_robot =  (cmdX_field * cos - cmdY_field * sin);   // forward/back
            double cmdY_robot =  cmdX_field * sin + cmdY_field * cos;   // strafe left/right

            // Heading control
            double angErr = angleError(targetAngle, robotTheta);
            double cmdTheta = thetaController.calculate(robotTheta + angErr);

            // Mecanum mixing
            double FL = cmdX_robot - cmdY_robot + cmdTheta;
            double BL = cmdX_robot + cmdY_robot - cmdTheta;
            double FR = cmdX_robot + cmdY_robot + cmdTheta;
            double BR = cmdX_robot - cmdY_robot - cmdTheta;

            telemetry.addData("Y movement: "    , cmdY_robot);
            telemetry.addData("X movement: "    , cmdX_robot);
            telemetry.addData("Theta movement: ", cmdTheta);


            // Normalize powers
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(FL), Math.abs(BL)),
                    Math.max(Math.abs(FR), Math.abs(BR))));
            FL /= max; BL /= max; FR /= max; BR /= max;

            // Apply to motors
            FLDrive.setPower(FL);
            BLDrive.setPower(BL);
            FRDrive.setPower(FR);
            BRDrive.setPower(BR);

            telemetry.addData("FL:", FL);
            telemetry.addData("BL:", BL);
            telemetry.addData("FR:", FR);
            telemetry.addData("BR:", BR);

            // Exit condition
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double distance = Math.hypot(dx, dy);

            telemetry.addData("X Distance:", dx);
            telemetry.addData("Y Distance:", dy);
            telemetry.addData("Target Distance:", distance);


            if (distance < posTolerance && Math.abs(targetAngle-robotTheta) < angleTolerance) break;

            telemetry.addData("In Position:",(distance < posTolerance));
            telemetry.addData("At Angle:",(Math.abs(angErr) < angleTolerance));

            telemetry.update();
        }

        // Stop motors
        FLDrive.setPower(0);
        BLDrive.setPower(0);
        FRDrive.setPower(0);
        BRDrive.setPower(0);
    }

    /**
     * Helper: shortest signed angular error in [-π, π].
     */
    private double angleError(double target, double current) {
        double error = target - current;
        error = (error + Math.PI) % (2.0 * Math.PI);
        if (error > Math.PI) error -= 2.0 * Math.PI;
        return error;
    }
}