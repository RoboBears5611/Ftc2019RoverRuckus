package team5611;

import ftclib.FtcGamepad;

/**
 * Created by Goerge on 9/30/2018.
 */

public abstract class Controls {
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    public Controls(FtcGamepad driverGamepad,FtcGamepad operatorGamepad){
        if(driverGamepad==null||operatorGamepad==null){
            throw new IllegalArgumentException("Null gamepad references were passed to this 'Controls' instance.  This is a very bad idea.");
        }
        this.driverGamepad = driverGamepad;
        this.operatorGamepad = operatorGamepad;
    }
    abstract double getTestMotorPower();
}
