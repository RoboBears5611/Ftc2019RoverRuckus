package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by FTC on 12/10/2016.
 */

@TeleOp(name = "ODSSensorReadout", group = "TEST")
@Disabled
public class ODSSensorReadout extends OpMode {
    OpticalDistanceSensor ODS;
    @Override
    public void init() {
        ODS = hardwareMap.opticalDistanceSensor.get("ODS");
    }

    @Override
    public void loop() {
        telemetry.addData("ODS",ODS.getLightDetected());
        telemetry.addData("ODSRAW",ODS.getRawLightDetected());
        telemetry.addData("ODSLINEAR",Math.pow(ODS.getRawLightDetected(),-0.5));
    }
}
