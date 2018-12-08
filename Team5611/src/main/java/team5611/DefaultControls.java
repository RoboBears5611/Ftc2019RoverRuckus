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
    double forward() {
        return driverGamepad.getLeftStickY();
    }


    @Override
    double turn() {
        return driverGamepad.getRightStickX();
    }

}
