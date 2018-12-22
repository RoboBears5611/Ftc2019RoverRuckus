package team5611;

import ftclib.FtcGamepad;

/**
 * Created by Goerge on 9/30/2018.
 */

public class DefaultControls extends Controls {


    public DefaultControls(FtcGamepad driverGamepad, FtcGamepad operatorGamepad) {
        super(driverGamepad, operatorGamepad);
    }

    @Override
    DriveType getDriveType() {
        return DriveType.Tank;
    }

    @Override
    double getLeftOrY(){
        return driverGamepad.getLeftStickY();

    }

    @Override
    double getRightOrTurn() {
        return driverGamepad.getRightStickY();
    }

    @Override
    public double armExtend() {
        return (driverGamepad.getButtons()>>12)&1 //adds 1 when active, 0 when not (DPAD_UP)
                -(driverGamepad.getButtons()>>13)&1; //subtracts 1 when activated, 0 when not (DPAD_DOWN)

    }

    @Override
    public double armRotate() {
        return (driverGamepad.getButtons()>>11)&1 //adds 1 when active, 0 when not (DPAD_RIGHT)
                -(driverGamepad.getButtons()>>10)&1; //subtracts 1 when activated, 0 when not (DPAD_LEFT)
    }

    @Override
    public boolean turboMode(){
        return ((driverGamepad.getButtons()>>7)&1)==1;
    }

    boolean collectorOn;
    boolean gamepadALast;
    @Override
    public double collector() {
        if(driverGamepad.getButton(FtcGamepad.GAMEPAD_A)||!gamepadALast){
            collectorOn = !collectorOn;
        }
        return collectorOn?1:0;
    }
}
