package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous (Boring) )",group="Autonomous")
@Disabled
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
        timeBasedMechanumDriveBase.addInstruction(0,0.5f,0,1500);
        GrabberLeft.setPosition(GrabberOpenPosition);
        GrabberRight.setPosition(GrabberOpenPosition);
        try {
            wait(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        timeBasedMechanumDriveBase.addInstruction(0,-0.5f,0,500);
        timeBasedMechanumDriveBase.addInstruction(0,0,-0.5f,1500);
        timeBasedMechanumDriveBase.addInstruction(0,0.5f,0,500);
        timeBasedMechanumDriveBase.executeInstructions();
    }
}
