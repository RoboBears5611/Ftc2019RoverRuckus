package team5611;

import ftclib.FtcGamepad;

/**
 * Created by Goerge on 9/30/2018.
 */

public abstract class Controls {
    public enum DriveType{
        Tank, Arcade
    }
    public class DriveArguments{
        public DriveArguments(){};
        public DriveArguments(DriveType driveType, double leftOrY, double rightOrTurn){
            this.driveType=driveType;
            this.leftOrY = leftOrY;
            this.rightOrTurn = rightOrTurn;
        }
        DriveType driveType;
        double leftOrY;
        double rightOrTurn;
    }

    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    public Controls(FtcGamepad driverGamepad,FtcGamepad operatorGamepad){
        if(driverGamepad==null||operatorGamepad==null){
            throw new IllegalArgumentException("Null gamepad references were passed to this 'Controls' instance.  This is a very bad idea.");
        }
        this.driverGamepad = driverGamepad;
        this.operatorGamepad = operatorGamepad;
    }
    public DriveArguments getDrive(){
        return new DriveArguments(getDriveType(),getLeftOrY(),getRightOrTurn());
    }
    abstract DriveType getDriveType();
    abstract double getLeftOrY();
    abstract double getRightOrTurn();

    public abstract double armExtend();

    public abstract double armRotate();

    public abstract boolean turboMode();

    public abstract double collector();
}
