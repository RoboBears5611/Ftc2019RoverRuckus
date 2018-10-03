package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by admin on 1/25/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by FTC on 11/21/2016.
 */

public class RobotControllerBase {
    private MechanumDriveBase mechanumDriveBase;
    private Servo GrabberLeft;
    private Servo GrabberRight;
    private Servo SuperWacker3000;
    private DcMotor Lift;
    private VoltageSensor PowerGage;
    private boolean VariableGrabberControl = false;
    private boolean GrabberState = false;
    private boolean LastToggleOpenClosed;
    private boolean LastToggleVariableControl;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private RobotControllerDriveControls controls;
    RobotControllerSettings settings;

    private double GrabberOpenPosition = 0.3;
    private double GrabberClosedPosition = 0.525;


    //    DcMotor ResettingMotor;
    public RobotControllerBase(Telemetry telemetry,  HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        mechanumDriveBase = new MechanumDriveBase(hardwareMap,telemetry);
    }

    public void init(RobotControllerSettings settings){
        this.settings = settings;
        mechanumDriveBase.init();
        GrabberLeft = hardwareMap.servo.get("GrabberLeft");
        GrabberLeft.setDirection(Servo.Direction.REVERSE);
        GrabberRight = hardwareMap.servo.get("GrabberRight");
        Lift = hardwareMap.dcMotor.get("Lift");
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SuperWacker3000 = hardwareMap.servo.get("SuperWacker3000");
        if(hardwareMap.dcMotorController.contains(settings.PowerGageName)){
            PowerGage = hardwareMap.voltageSensor.get(settings.PowerGageName);
        }
        UpdateGrabbers();
    }

    public void start(){
        SuperWacker3000.setPosition(0.25);
        stopAllMotors();
    }


    public void stopAllMotors(){
        mechanumDriveBase.stopAllMotors();
    }

    public void loop(RobotControllerDriveControls controls){
        this.controls = controls;

        mechanumDriveBase.move(controls.moveX,controls.moveY,controls.moveTurn,controls.moveForceFull,PowerGage==null?0:PowerGage.getVoltage());
        grabberOpenAndClose();
        // VariableGrabberControl();
        //EmergencyResets();
        Lift.setPower(controls.liftPower);


        telemetry.addData("Lift Target Pos",Lift.getTargetPosition());
        telemetry.addData("RPos",GrabberRight.getPosition());
        telemetry.addData("LPos",GrabberLeft.getPosition());
    }

    public void updateSettings(RobotControllerSettings settings){
        this.settings = settings;
        this.GrabberClosedPosition = settings.GrabberClosedPosition;
        this.GrabberOpenPosition = settings.GrabberOpenPosition;
    }

    private void grabberOpenAndClose(){
        if(VariableGrabberControl){
            return;
        }
        ChangeOpenClosePos(
                controls.grabberOpenCloseAdjustment);
        if(controls.grabberToggleOpenClosed&&!LastToggleOpenClosed) {                                                  //If the A button is pushed
            GrabberState = !GrabberState;
            UpdateGrabbers();
        }
        LastToggleOpenClosed = controls.grabberToggleOpenClosed;
    }

    private void ChangeOpenClosePos(double amount){
        if(GrabberState){
            GrabberClosedPosition += amount;
        }else{
            GrabberOpenPosition += amount;
        }
    }

    private void UpdateGrabbers(){
        GrabberLeft.setPosition(GrabberState?GrabberClosedPosition:GrabberOpenPosition);
        GrabberRight.setPosition(GrabberState?GrabberClosedPosition:GrabberOpenPosition);
    }
    private void VariableGrabberControl(){
        if(controls.grabberToggleVariableControl&&!LastToggleVariableControl){
            VariableGrabberControl = !VariableGrabberControl;
        }
        if(VariableGrabberControl){
            GrabberLeft.setPosition(controls.grabberVariableControlPosition);
            GrabberRight.setPosition(controls.grabberVariableControlPosition);
        }
        LastToggleVariableControl = controls.grabberToggleVariableControl;
    }
//Unnecessary until we get a Encoder-Enabled Motor that could need resetting
//    public void EmergencyResets(){
//        if(gamepad2.dpad_up){
//
//        }
//        if(ResettingMotor!=null){
//            ResettingMotor.setPower((gamepad2.left_trigger/2)-(gamepad2.right_trigger/2));
//        }
//        if(gamepad2.dpad_down&&ResettingMotor!=null){
//            DcMotor.RunMode originalMode = ResettingMotor.getMode();
//            ResettingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            while(ResettingMotor.isBusy());
//            ResettingMotor.setMode(originalMode);
//            ResettingMotor = null;
//        }
//        telemetry.addData("ResettingMotor",ResettingMotor==null?"None":ResettingMotor);
//        if(ResettingMotor!=null){
//            telemetry.addData("ResettingMotor Power",ResettingMotor.getPower());
//        }
//    }
}

