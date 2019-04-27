package team5611;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Date;

import ftclib.FtcOpMode;
import trclib.TrcRobot;

@Autonomous(name="Full Autonomous",group = "Auto")
public class FtcAutoFull extends FtcOpMode {
    CmdAutoFull opMode;
    Date[] lastTimes;

    @Override
    public void initRobot() {
        opMode = new CmdAutoFull(new Robot5611(TrcRobot.RunMode.AUTO_MODE),0);

    }

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode) {
        lastTimes = new Date[2];
        lastTimes[0] = new Date();
    }

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode) {

    }

    @Override
    public void runContinuous(double elapsedTime) {
        lastTimes[1] = new Date();
        opMode.cmdPeriodic(new Date().getTime()-lastTimes[0].getTime());
        lastTimes[0] = lastTimes[1];
    }
}
