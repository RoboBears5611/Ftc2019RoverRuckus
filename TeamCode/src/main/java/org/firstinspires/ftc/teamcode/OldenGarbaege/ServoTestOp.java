package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test:  Servo",group = "Test")
@Disabled
public class ServoTestOp extends OpMode {
    private Servo s1;

    private double s1Pos;

    public void init(){
        s1 = hardwareMap.servo.get("servo1");
    }
    public void start(){
        s1.setPosition(0);
        s1Pos = 0;
    }

    public void loop(){
        s1Pos = gamepad1.left_trigger;

        s1.setPosition(s1Pos);
        telemetry.addData("s1Pos",s1Pos);
    }
}