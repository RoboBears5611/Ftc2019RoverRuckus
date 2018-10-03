package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Goerge on 9/30/2018.
 */

public abstract class Controls {
    protected Gamepad driverGamepad;
    protected Gamepad operatorGamepad;
    public Controls(Gamepad driverGamepad,Gamepad operatorGamepad){
        this.driverGamepad = driverGamepad;
        this.operatorGamepad = operatorGamepad;
    }
    abstract double getTestMotorPower();
}
