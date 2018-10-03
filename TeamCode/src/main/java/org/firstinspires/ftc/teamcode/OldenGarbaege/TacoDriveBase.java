package org.firstinspires.ftc.teamcode.OldenGarbaege;

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

public class TacoDriveBase {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private final static String LFMotorName = "LFMotor";
    private final static String RFMotorName = "RFMotor";
    private final static String LMMotorName = "LMMotor";
    private final static String RMMotorName = "RMMotor";

    private DcMotor LFMotor;
    private boolean LFMotorConnected;
    private DcMotor RFMotor;
    private boolean RFMotorConnected;
    private DcMotor LMMotor;
    private boolean LMMotorConnected;
    private DcMotor RMMotor;
    private boolean RMMotorConnected;
    public double LeftTaco;
    public double RightTaco;

    public TacoDriveBase(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }


    public void setBothMotors(double value) {
        LeftTaco = value; RightTaco = value;
        updateMotors();
    }
    public void updateMotors(){

        if(LFMotorConnected){
            LFMotor.setPower(LeftTaco);
        }
        if(LMMotorConnected){
            LMMotor.setPower(LeftTaco);
        }
        if(RFMotorConnected){
            RFMotor.setPower(RightTaco);
        }
        if(RMMotorConnected){
            RMMotor.setPower(RightTaco);
        }
        telemetry.addData("LeftTaco",LeftTaco);
        telemetry.addData("RightTaco",RightTaco);
    }
    public void initializeMotors() {
        Set<Map.Entry<String,DcMotor>> motors =  hardwareMap.dcMotor.entrySet();
        List<String> names = new LinkedList<>();
        for(Map.Entry<String,DcMotor> entry : motors){
            names.add(entry.getKey());
        }
        telemetry.addData("ConnectedMotors",names);
        if(names.contains(LFMotorName)){
            LFMotor = hardwareMap.dcMotor.get(LFMotorName);
            LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            LFMotorConnected = true;
        }else{
            LFMotorConnected = false;
        }
        if(names.contains(RFMotorName)){
            RFMotor = hardwareMap.dcMotor.get(RFMotorName);
            RFMotorConnected = true;
        }else{
            RFMotorConnected = false;
        }
        if(names.contains(LMMotorName)){
            LMMotor = hardwareMap.dcMotor.get(LMMotorName);
            LMMotorConnected = true;
        }else{
            LMMotorConnected = false;
        }
        if(names.contains(RMMotorName)){
            RMMotor = hardwareMap.dcMotor.get(RMMotorName);
            RMMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            RMMotorConnected = true;
        }else{
            RMMotorConnected = false;
        }
        telemetry.addData("LFMotorConnected (named "+LFMotorName+")",LFMotorConnected);
        telemetry.addData("RFMotorConnected (named "+RFMotorName+")",RFMotorConnected);
        telemetry.addData("LMMotorConnected (named "+LMMotorName+")",LMMotorConnected);
        telemetry.addData("RMMotorConnected (named "+RMMotorName+")",RMMotorConnected);
        telemetry.update();
    }

}
