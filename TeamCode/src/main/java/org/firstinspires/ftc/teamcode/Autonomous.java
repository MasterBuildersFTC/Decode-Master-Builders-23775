package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", preselectTeleOp = "FCDrivingITD")
public class Autonomous extends LinearOpMode {

    /*private static final double kPx = 0.056, kIx = 0.0, kDx = 0.007;
    private static final double kPy = 0.056, kIy = 0.0, kDy = 0.0059;
    private static final double kPt = 0.02, kIt = -0.0019, kDt = 0.01;

    private final PIDController xController = new PIDController(kPx, kIx, kDx);
    private final PIDController yController = new PIDController(kPy, kIy, kDy);
    private final PIDController thetaController = new PIDController(kPt, kIt, kDt);*/

    private DcMotor FrontLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;

    //private DcMotor FrontIntake;
    //private DcMotor BackIntake;
    //GoBildaPinpointDriver Odometry; // Declare OpMode member for the Odometry Computer

    public void runOpMode() {
        //Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        //Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLDrive");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRDrive");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLDrive");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRDrive");

        // A default TargetPosition must be set
        /*FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        FrontLeftMotor.setTargetPosition(0);
        FrontRightMotor.setTargetPosition(0);
        BackLeftMotor.setTargetPosition(0);
        BackRightMotor.setTargetPosition(0);

        //Making Sure wheels are turning in the right direction
        //port 0
        FrontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //port 1
        BackLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //port 2
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //port 3
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*FrontIntake = hardwareMap.get(DcMotor.class, "FrontIntake");
        BackIntake = hardwareMap.get(DcMotor.class, "BackIntake");
        FrontIntake.setDirection(DcMotor.Direction.REVERSE);
        BackIntake.setDirection(DcMotor.Direction.REVERSE);*/

        /*Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Odometry.setOffsets(-30, 0);
        Odometry.recalibrateIMU();
        Odometry.resetPosAndIMU();
        Odometry.recalibrateIMU();
        Odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        Odometry.setPosition(Pose2D.from);*/

        waitForStart();

        //Drive_Controls(300,300,0,50,5,300000);

        // Run Autonomous
        autonomousRun(30000);
    }

    private void autonomousRun(long timeoutMillis) {

        long start = System.currentTimeMillis();

        // Keep running while BOTH Autonomous is active AND we aren't timed out
        //while(opModeIsActive() && (System.currentTimeMillis() - start > timeoutMillis)) {

            // Get the current position
            /*double robotX = Odometry.getPosX();
            double robotY = Odometry.getPosY();
            double robotTheta = Odometry.getHeading();*/ // radians

            // Keep Odometry updated (we may not even need this)
            /*Pose2D newPosition = new Pose2D(DistanceUnit.MM, robotX+100, robotY+0, AngleUnit.DEGREES, 0);
            Odometry.setPosition(newPosition);
            Odometry.update();*/

            // Set Where To Go: 10mm (1.37 ticks == 1mm)
            FrontLeftMotor.setTargetPosition(50);
            FrontRightMotor.setTargetPosition(50);
            BackLeftMotor.setTargetPosition(50);
            BackRightMotor.setTargetPosition(50);

            // MOVE: % (move slow)
            FrontLeftMotor.setPower(0.01);
            FrontRightMotor.setPower(0.01);
            BackLeftMotor.setPower(0.01);
            BackRightMotor.setPower(0.01);

            // Wait until it is done moving
            /*while(FrontLeftMotor.isBusy() && FrontRightMotor.isBusy() &&
                    BackLeftMotor.isBusy() && BackRightMotor.isBusy()) {
                /* Do Nothing
            }*/

            // Run for 3 seconds
            start = System.currentTimeMillis();
            while((start + 3000) > System.currentTimeMillis()) {
                telemetry.addData("Status", "Waiting");
                telemetry.update();
                try {
                    Thread.sleep(1000);
                } catch (Throwable t) {
                    // Do Nothing
                }
            }

            // Stop all Motors
            FrontLeftMotor.setPower(0);
            FrontRightMotor.setPower(0);
            BackLeftMotor.setPower(0);
            BackRightMotor.setPower(0);



        //}
    }

    /*private void Drive_Controls(double targetX, double targetY, double targetAngle, double posTolerance, double angleTolerance, long timeoutMillis) {

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

            double Maximum = 1;//(1-Math.exp(.01*RobotDistance));

            double powerScaling = Maximum / RobotDistance;

            telemetry.addData("Scaling", powerScaling);

            double xPower = xDistance * powerScaling;
            double yPower = yDistance * powerScaling;

            telemetry.addData ("x Power", xPower);
            telemetry.addData ("y Power", yPower);

        }
    }*/
}

