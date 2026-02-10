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

    private static final double kPx = 0.056, kIx = 0.0, kDx = 0.007;
    private static final double kPy = 0.056, kIy = 0.0, kDy = 0.0059;
    private static final double kPt = 0.02, kIt = -0.0019, kDt = 0.01;

    private final PIDController xController = new PIDController(kPx, kIx, kDx);
    private final PIDController yController = new PIDController(kPy, kIy, kDy);
    private final PIDController thetaController = new PIDController(kPt, kIt, kDt);
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private DcMotor Intake;
    private DcMotor LiftSystem;
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

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        LiftSystem = hardwareMap.get(DcMotor.class, "LiftSystem");
        Intake.setDirection(DcMotor.Direction.REVERSE);
        LiftSystem.setDirection(DcMotor.Direction.REVERSE);

        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Odometry.setOffsets(-30, 0);
        Odometry.recalibrateIMU();
        Odometry.resetPosAndIMU();
        Odometry.recalibrateIMU();
        Odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //Odometry.setPosition(Pose2D.from);

        waitForStart();

        Drive_Controls(0,300,0,75,5,300000);
    }

    private void Drive_Controls(double targetX, double targetY, double targetAngle, double posTolerance, double angleTolerance, long timeoutMillis) {

        long start = System.currentTimeMillis();

        angleTolerance = Math.toRadians(angleTolerance);
        targetAngle = Math.toRadians(targetAngle);

        while (opModeIsActive()) {
            telemetry.update();
            Odometry.update();
            // Timeout safety
            if (System.currentTimeMillis() - start > timeoutMillis) break;

            // Read odometry (replace with your actual odometry calls!)
            double robotX = Odometry.getPosX();
            double robotY = Odometry.getPosY();
            double robotTheta = Odometry.getHeading(); // radians

            telemetry.addData("robotX: ", robotX);
            telemetry.addData("robotY: ", robotY);
            telemetry.addData("robotTheta:", robotTheta);

            double xDistance = targetX - robotX;
            double yDistance = targetY - robotY;
            double thetaDistance = Math.toRadians(targetAngle) - robotTheta;

            telemetry.addData("x Distance", xDistance);
            telemetry.addData("y Distance", yDistance);

            double RobotDistance = Math.hypot(xDistance, yDistance);

            double Maximum = (1-Math.exp(-.01*RobotDistance));

            telemetry.addData("Maximum", Maximum);

            double powerScaling = Maximum / RobotDistance;

            telemetry.addData("Scaling", powerScaling);

            double xPower = (xDistance*powerScaling);
            double yPower = (yDistance*powerScaling);

            telemetry.addData ("x Power", xPower);
            telemetry.addData ("y Power", yPower);

            double rotX = xPower * Math.cos(-robotTheta) - yPower * Math.sin(-robotTheta);
            double rotY = xPower * Math.sin(-robotTheta) + yPower * Math.cos(-robotTheta);

            double denominator = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(thetaDistance), 1);
            double FLDrivePower = (rotY - rotX + thetaDistance) / denominator;
            double BLDrivePower = (rotY + rotX + thetaDistance) / denominator;
            double FRDrivePower = (rotY + rotX - thetaDistance) / denominator;
            double BRDrivePower = (rotY - rotX - thetaDistance) / denominator;

            telemetry.addData("Motor 1", FLDrivePower);
            telemetry.addData("Motor 2", BLDrivePower);
            telemetry.addData("Motor 3", FRDrivePower);
            telemetry.addData("Motor 4", BRDrivePower);

            FLDrive.setPower(FLDrivePower);
            BLDrive.setPower(BLDrivePower);
            FRDrive.setPower(FRDrivePower);
            BRDrive.setPower(BRDrivePower);

            if (posTolerance > RobotDistance){
                break;
            }
        }
    }
}

