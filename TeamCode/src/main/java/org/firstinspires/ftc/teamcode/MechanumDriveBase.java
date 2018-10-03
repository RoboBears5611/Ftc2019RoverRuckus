package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;


/**
 * Created by FTC on 12/13/2016.
 */

public class MechanumDriveBase {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private final static String LFMotorDefaultName = "LFMotor";
    private final static String RFMotorDefaultName = "RFMotor";
    private final static String LBMotorDefaultName = "LBMotor";
    private final static String RBMotorDefaultName = "RBMotor";

    private final static boolean LFMotorDefaultReversed = true;
    private final static boolean RFMotorDefaultReversed = false;
    private final static boolean LBMotorDefaultReversed = true;
    private final static boolean RBMotorDefaultReversed = false;

    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;

    private final static float NormalDriveMultiplier = 0.5f;
    private final static double SlowdownVoltage = 8;
    private final static double SlowdownMultiplier = 0.1;
    private final static double CutoffVoltage = 6;

    public MechanumDriveBase(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }


    public void init() {
        init(   LFMotorDefaultName, LFMotorDefaultReversed,
                RFMotorDefaultName, RFMotorDefaultReversed,
                LBMotorDefaultName, LBMotorDefaultReversed,
                RBMotorDefaultName, RBMotorDefaultReversed);
    }
    public void init(String LFMotorName, boolean LFMotorReversed,
                     String RFMotorName, boolean RFMotorReversed,
                     String LBMotorName, boolean LBMotorReversed,
                     String RBMotorName, boolean RBMotorReversed) {
        RFMotor = hardwareMap.dcMotor.get(RFMotorName);
        LFMotor = hardwareMap.dcMotor.get(LFMotorName);
        RBMotor = hardwareMap.dcMotor.get(RBMotorName);
        LBMotor = hardwareMap.dcMotor.get(LBMotorName);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setDirection(RFMotorReversed? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD);
        LFMotor.setDirection(LFMotorReversed? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD);
        RBMotor.setDirection(RBMotorReversed? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD);
        LBMotor.setDirection(LBMotorReversed? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void stopAllMotors(){
        LFMotor.setPower(0);
        RFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void move(float x, float y, float turn){
        move(x,y,turn,false);
    }
    public void move(float x, float y, float turn, boolean forceFullPower){
        move(x,y,turn,forceFullPower,0);
    }
    public void move(float x, float y, float turn, boolean forceFullPower, double voltage){
        telemetry.addData("Drive X",x);
        telemetry.addData("Drive Y",y);
        telemetry.addData("Drive Turn",turn);

        //good fun.
        double r = Math.hypot(x, y); //Get how far the joystick is pushed
        double robotAngle = Math.atan2(-y, -x) - Math.PI / 4; //Rotate everything 45 degrees, since the wheels operate on tires with an exactly 45 degree offset from forwards and backwards on the robot

        double motorMultiplier;
        if(forceFullPower){
            motorMultiplier = 1; //Disregard all concerns of voltage or control when force full is on.
        }else{
            float NormalWithStrafeJuice = Math.abs(x)>0.5f?1f:NormalDriveMultiplier;
            if(voltage == 0){
                motorMultiplier = NormalWithStrafeJuice; //If the voltage is set to 0, that's an indicator that a) something in the volrage sensing system is ascue, and we
                //  ... need to disregard it, or no voltage was ever actually provided, in which case we also don't care what the voltage is.
            }else{
                if(voltage<=CutoffVoltage){
                    motorMultiplier = 0;
                }else if(voltage<=SlowdownVoltage){
                    motorMultiplier = SlowdownMultiplier;
                }else{
                    motorMultiplier = NormalWithStrafeJuice;
                }
            }
        }

        final double LF = (r * Math.cos(robotAngle) + turn)*motorMultiplier; //Calculate just how much speed each wheel should get in relation to each other (as stated by some Trig!) and then multiply that by how much power we actually want (Sin and Cos won't give us that, since they are just working with angles)
        final double RF = (r * Math.sin(robotAngle) - turn)*motorMultiplier ;//We then subtract or add to the power, according to the wheels orientation to the rest of the bot.  This change causes the robot to turn.
        final double LB = (r * Math.sin(robotAngle) + turn)*motorMultiplier;
        final double RB = (r * Math.cos(robotAngle) - turn)*motorMultiplier;

        LFMotor.setPower(LF);
        telemetry.addData("LFMotor",LF);
        RFMotor.setPower(RF);
        telemetry.addData("RFMotor",RF);
        LBMotor.setPower(LB);
        telemetry.addData("LBMotor",LB);
        RBMotor.setPower(RB);
        telemetry.addData("RBMotor",RB);
    }

}

