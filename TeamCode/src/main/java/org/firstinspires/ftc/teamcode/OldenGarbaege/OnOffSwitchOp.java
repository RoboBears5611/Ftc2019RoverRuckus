package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC on 12/1/2016.
 */

@TeleOp(name = "OnOffSwitchOp",group = "TEST")
@Disabled
public class OnOffSwitchOp extends OpMode {
    DcMotor Drive;
    @Override
    public void init() {
        Drive = hardwareMap.dcMotor.get("Drive");

    }

    @Override
    public void loop() {
        Drive.setPower(gamepad1.left_stick_y);
    }
}
