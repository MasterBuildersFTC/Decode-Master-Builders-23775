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
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "FCDrivingITD", group = "LinearOpMode")
public class Field_Centric_TeleOp extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime();
    GoBildaPinpointDriver Odometry; // Declare OpMode member for the Odometry Computer
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private DcMotorEx TopOuttake;
    private DcMotorEx BottomOuttake;
    private DcMotor FrontIntake;
    private DcMotor BackIntake;
    private Servo IndexRamp;
    private CRServo IndexRevolver;
    private Limelight3A Limelight;

    double IndexRevolverPosition = 0;
    boolean FormerIndex = false;
    double targetHeading = 0.0; // For heading lock
    int RetractionTime = 0;
    double OuttakeVelocity=0;

    boolean formerA = false;
    boolean formerB = false;
    boolean formerX = false;
    boolean formerY = false;


    @Override
    public void runOpMode() {
        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        Limelight.pipelineSwitch(0);

        Limelight.start();

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
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
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

        TopOuttake = hardwareMap.get(DcMotorEx.class, "TopOuttake");
        BottomOuttake = hardwareMap.get(DcMotorEx.class, "BottomOuttake");
        TopOuttake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BottomOuttake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FrontIntake = hardwareMap.get(DcMotor.class, "FrontIntake");
        BackIntake = hardwareMap.get(DcMotor.class, "BackIntake");
        FrontIntake.setDirection(DcMotor.Direction.REVERSE);
        BackIntake.setDirection(DcMotor.Direction.REVERSE);

        IndexRamp = hardwareMap.get(Servo.class, "IndexRamp");

        IndexRevolver = hardwareMap.get(CRServo.class, "IndexRevolver");
        //IndexRevolver.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
            Index_Controls();
            Launch_System();
            Intake_System();
            telemetry.update();
        }
    }
    //General OpMode Specific Functions
    private void Intake_System() {
        double IntakePower = gamepad1.left_trigger;

        if (gamepad1.left_bumper)
            IntakePower = .54;

        FrontIntake.setPower(IntakePower);
        BackIntake.setPower(IntakePower);
        
        telemetry.addData("Intake Power", IntakePower);
    }
    private void Launch_System() {
        double goalDistance = ((129.9-35) / Math.tan(Math.toRadians(22.5+Limelight.getLatestResult().getTy())));
        double goalAngleDifference = Limelight.getLatestResult().getTx();

        if (gamepad2.a && !formerA)
            OuttakeVelocity -= .1;
        if (gamepad2.b && !formerB)
            OuttakeVelocity -= .01;
        if (gamepad2.x && !formerX)
            OuttakeVelocity += .01;
        if (gamepad2.y && !formerY)
            OuttakeVelocity += .1;

        /*if (gamepad2.right_bumper) {
            OuttakeVelocity = 0;
        }*/

        formerA = gamepad2.a;
        formerB = gamepad2.b;
        formerX = gamepad2.x;
        formerY = gamepad2.y;

        //OuttakeVelocity = (0.00000275155*goalDistance*goalDistance) - (0.000903248*goalDistance)+0.670476;

        TopOuttake.setPower(OuttakeVelocity);
        BottomOuttake.setPower(OuttakeVelocity);
        
        telemetry.addData("OuttakeVelocity", OuttakeVelocity);
        telemetry.addData("Distance from back of goal:", goalDistance);
        telemetry.addData("angle difference:", goalAngleDifference);

        
    }
    private void Index_Controls() {
        Index_Ramp();
        IndexRevolver();
    }
    private void IndexRevolver() {
        /*if (gamepad1.left_bumper && FormerIndex) {
            IndexRevolverPosition += (.2);
            FormerIndex = true;
        }

        if (gamepad1.right_bumper && FormerIndex) {
            IndexRevolverPosition -= (.2);
            FormerIndex = true;
        }

        if (!gamepad1.right_bumper && !gamepad1.left_bumper);{
            FormerIndex = false;
        }*/

        IndexRevolver.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        telemetry.addData("IndexRevolver Position: ", IndexRevolverPosition);
    }
    private void Index_Ramp() {
        double ElapsedTime = runTime.seconds();

        if (gamepad2.right_bumper) {
            IndexRamp.setPosition(1);
            RetractionTime = (int) (ElapsedTime+2);
        }
        if (ElapsedTime > RetractionTime) {
            IndexRamp.setPosition(.87);
        }

        double IndexRampAngle = IndexRamp.getPosition();
        telemetry.addData("Scissor Lift Angle: ", IndexRampAngle);
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

        double rx = -gamepad1.right_stick_x;

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
        double FLDrivePower = (rotY + rotX - rx) / denominator;
        double BLDrivePower = (rotY - rotX - rx) / denominator;
        double FRDrivePower = (rotY - rotX + rx) / denominator;
        double BRDrivePower = (rotY + rotX + rx) / denominator;

        //FLDrivePower =0;
        //BLDrivePower =0;
        //FRDrivePower =0;
        //BRDrivePower =0;
//
        //if (gamepad1.a) {
        //    FLDrivePower = 1;
        //}
        //if (gamepad1.b) {
        //    BLDrivePower = 1;
//
        //}
        //if (gamepad1.x) {
        //    FRDrivePower = 1;
        //}
        //if (gamepad1.y) {
        //    BRDrivePower = 1;
        //}

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
