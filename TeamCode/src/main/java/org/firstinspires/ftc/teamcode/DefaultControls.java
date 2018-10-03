package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Goerge on 9/30/2018.
 */

public class DefaultControls extends Controls {

    public DefaultControls(Gamepad driverGamepad, Gamepad operatorGamepad) {
        super(driverGamepad, operatorGamepad);
    }

    @Override
    double getTestMotorPower() {
        return driverGamepad.left_stick_y;
    }
}
