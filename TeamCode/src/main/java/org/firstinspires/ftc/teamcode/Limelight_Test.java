package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Limelight Test", group = "LinearOpMode")
public class Limelight_Test extends LinearOpMode {
    private Limelight3A Limelight;

    public void runOpMode() {
        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        Limelight.pipelineSwitch(0);

        Limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            double distance = ((h2-h1) / Math.tan(22.5+Limelight.getLatestResult().getTy()));
            double angleDiffernce = Limelight.getLatestResult().getTx();

            telemetry.addData("angle difference:", angleDiffernce);
            telemetry.addData("Distance from back of goal:", distance);
            telemetry.update();
        }
    }
}
