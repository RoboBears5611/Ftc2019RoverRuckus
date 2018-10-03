package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by FTC on 11/21/2016.
 */

@TeleOp(name = "TacoDrive",group = "drive")
@Disabled
public class TacoDriveOp extends OpMode {
    TacoDriveBase tacoDriveBase;
    private Telemetry telemetryFinal;
    private HardwareMap hardwareMapFinal;
    private Gamepad gamepad1Final;
    private Gamepad gamepad2Final;
    public TacoDriveOp(){
        telemetryFinal = telemetry;
        hardwareMapFinal = hardwareMap;
    }
    public TacoDriveOp(HardwareMap newHardwareMap, Telemetry newTelemetry){
        telemetryFinal = newTelemetry;
        hardwareMapFinal = newHardwareMap;
    }
    @Override
    public void init(){
        tacoDriveBase = new TacoDriveBase(hardwareMapFinal,telemetryFinal);
        tacoDriveBase.initializeMotors();
    }

    @Override
    public void start(){
        tacoDriveBase.setBothMotors(0);
    }

    public void loop(Gamepad newGamepad1, Gamepad newGamepad2){
        gamepad1Final = newGamepad1;
        gamepad2Final = newGamepad2;
        loopCode();
    }

    @Override
    public void loop(){
        gamepad1Final = gamepad1;
        gamepad2Final = gamepad2;
        loopCode();
    }
    private void loopCode(){
        tacoDriveBase.RightTaco = -gamepad1Final.right_stick_y*-1;
        tacoDriveBase.LeftTaco = -gamepad1Final.left_stick_y*-1;
        tacoDriveBase.updateMotors();
    }
}