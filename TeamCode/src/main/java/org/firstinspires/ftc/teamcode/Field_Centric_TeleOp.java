package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "FCDrivingITD", group = "LinearOpMode")
public class Field_Centric_TeleOp extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime();
    GoBildaPinpointDriver Odometry; // Declare OpMode member for the Odometry Computer
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private DcMotorEx LeftLauncher;
    private DcMotorEx RightLauncher;
    private DcMotor RightIntake;
    private Servo ScissorLift;
    private CRServo Revolver;

    double RevolverPosition = 0;
    boolean FormerIndex = false;
    double targetHeading = 0.0; // For heading lock

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

        LeftLauncher = hardwareMap.get(DcMotorEx.class, "LeftLauncher");
        RightLauncher = hardwareMap.get(DcMotorEx.class, "RightLauncher");
        LeftLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        RightLauncher.setDirection(DcMotorEx.Direction.REVERSE);

        RightIntake = hardwareMap.get(DcMotor.class, "RightIntake");
        RightIntake.setDirection(DcMotor.Direction.REVERSE);

        ScissorLift = hardwareMap.get(Servo.class, "ScissorLift");

        Revolver = hardwareMap.get(CRServo.class, "Revolver");
        //Revolver.setPwmRange(new PwmControl.PwmRange(500, 2500));

        waitForStart();
        runTime.reset();

        //REMOVE WHEN AUTONOMOUS IS IN PLACE
        /*
        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Odometry.setOffsets(-68.0, 0);
        Odometry.recalibrateIMU();
        Odometry.resetPosAndIMU();
        Odometry.recalibrateIMU();
        Odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
*/
        while (opModeIsActive()) {
            Drive_Controls();
            Revolver_Controls();
            Launch_System();
            Intake_System();
            telemetry.update();
        }
    }
    //General OpMode Specific Functions
    private void Intake_System() {
        RightIntake.setPower(gamepad1.left_trigger);
        telemetry.addData("Intake Power", gamepad1.left_trigger);
    }
    private void Launch_System() {
        LeftLauncher.setPower(gamepad1.right_trigger/2);
        RightLauncher.setPower(gamepad1.right_trigger/2);

        telemetry.addData("Launcher Power", gamepad1.right_trigger/2);
        
    }
    private void Revolver_Controls() {
        Scissor_Lift();
        Revolver();
    }
    private void Revolver() {
        /*if (gamepad1.left_bumper && FormerIndex) {
            RevolverPosition += (.2);
            FormerIndex = true;
        }

        if (gamepad1.right_bumper && FormerIndex) {
            RevolverPosition -= (.2);
            FormerIndex = true;
        }

        if (!gamepad1.right_bumper && !gamepad1.left_bumper);{
            FormerIndex = false;
        }*/
        Revolver.setPower(0);
        if (gamepad1.left_bumper) {
            Revolver.setPower(1);
        }
        if (gamepad1.right_bumper) {
            Revolver.setPower(-1);
        }

        telemetry.addData("Revolver Position: ", RevolverPosition);
    }
    private void Scissor_Lift() {

        if (gamepad1.y) {
            ScissorLift.setPosition(.4);
        }
        if (gamepad1.a) {
            ScissorLift.setPosition(.7);
        }

        double ScissorLiftAngle = ScissorLift.getPosition();
        telemetry.addData("Scissor Lift Angle: ", ScissorLiftAngle);
    }
    private void Drive_Controls() {

        //Inspired by https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html and used microsoft copilot to help refine code
        if (gamepad1.back) {
            Odometry.resetPosAndIMU();
            Odometry.recalibrateIMU();
        }

        Odometry.update();
        double y = -(gamepad1.left_stick_y);
        double x = (gamepad1.left_stick_x);


        double botHeading = Odometry.getHeading();
        telemetry.addData("Yaw: ", botHeading);

        double rx = gamepad1.right_stick_x;
        double deadband = 0.1;

        if (rx != 0) {
            targetHeading = botHeading;
        }

        if (rx ==0) {
            rx = targetHeading - botHeading;
        }
         
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
    }
}
