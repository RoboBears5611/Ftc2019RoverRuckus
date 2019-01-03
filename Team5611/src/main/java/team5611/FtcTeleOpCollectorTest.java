package team5611;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;

@TeleOp(name="Collector Test", group="TeleOp")
public class FtcTeleOpCollectorTest extends FtcOpMode {
    FtcDcMotor CollectorMotor;


    @Override
    public void initRobot() {
        CollectorMotor = new FtcDcMotor(RobotInfo.CollectorMotorName);
   }

    @Override
    public void runPeriodic(double elapsedTime) {
        double value = gamepad1.right_stick_y;
        CollectorMotor.set(value);
    }
}
