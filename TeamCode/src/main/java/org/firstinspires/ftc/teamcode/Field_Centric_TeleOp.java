package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FCDrivingITD", group = "LinearOpMode")
public class Field_Centric_TeleOp {
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

        Drive0 = hardwareMap.get(DcMotor.class, "Drive0");
        Drive1 = hardwareMap.get(DcMotor.class, "Drive1");
        Drive2 = hardwareMap.get(DcMotor.class, "Drive2");
        Drive3 = hardwareMap.get(DcMotor.class, "Drive3");
    }
}
