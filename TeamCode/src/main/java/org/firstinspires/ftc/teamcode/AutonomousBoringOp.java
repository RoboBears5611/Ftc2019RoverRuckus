package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous (Boring) )",group="Autonomous")

public class AutonomousBoringOp extends LinearOpMode {
   TimeBasedMechanumDriveBase timeBasedMechanumDriveBase;
    private Servo GrabberLeft;
    private Servo GrabberRight;
    private double GrabberOpenPosition = 0.3;
    private double GrabberClosedPosition = 0.525;

    @Override
    public void runOpMode(){
        timeBasedMechanumDriveBase = new  TimeBasedMechanumDriveBase(hardwareMap,telemetry);
        GrabberLeft = hardwareMap.servo.get("GrabberLeft");
        GrabberLeft.setDirection(Servo.Direction.REVERSE);
        GrabberRight = hardwareMap.servo.get("GrabberRight");

        waitForStart();

        GrabberLeft.setPosition(GrabberClosedPosition);
        GrabberRight.setPosition(GrabberClosedPosition);
        timeBasedMechanumDriveBase.move(0,0.5f,0,1500).Complete();
        GrabberLeft.setPosition(GrabberOpenPosition);
        GrabberRight.setPosition(GrabberOpenPosition);
        try {
            wait(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        timeBasedMechanumDriveBase.move(0,-0.5f,0,500).Complete();
        timeBasedMechanumDriveBase.move(0,0,-0.5f,1500).Complete();
        timeBasedMechanumDriveBase.move(0,0.5f,0,500).Complete();

    }
}
