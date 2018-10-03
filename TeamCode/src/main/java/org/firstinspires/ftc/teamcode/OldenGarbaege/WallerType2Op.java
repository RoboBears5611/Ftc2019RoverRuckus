package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by FTC on 11/29/2016.
 */

@Autonomous(name="WallerType2Op", group = "Autonomous")
@Disabled
public class WallerType2Op extends LinearOpMode{

    OpticalDistanceSensor Waller;
    private final static double TargetWallLightRaw = 0.5;
    private final static double LineFollowSpeed = 0.5;
    private final static double SearchingForWallSpeed = 0.5;
    private final static double CloseLimit = 2.85;
    private final static double FarLimit = 10;

    //Calculated From provided CloseLimit, FarLimit, and LineFollowSpeed.  This computationally determines the Linear Function Variables necessary to follow a wall, automatically.
    //Automatically coming up with linear variables because it allows us to tinker with the Speed and Limit variables and not have to manually run it through a bunch of boring algebraic stuff.
    private final static double LeftSlope = LineFollowSpeed/(CloseLimit-FarLimit);
    private final static double LeftIntercept = -CloseLimit*LeftSlope+LineFollowSpeed;
    private final static double RightSlope = -LineFollowSpeed/(CloseLimit-FarLimit);
    private final static double RightIntercept = -CloseLimit*RightSlope;

    private String currentStatus;
    private TacoDriveBase tacoDriveBase;

    @Override
    public void runOpMode() throws InterruptedException{
        setStatus("Initializing");
        updateTelemetry();
        tacoDriveBase = new TacoDriveBase(hardwareMap,telemetry);
        tacoDriveBase.initializeMotors();
        tacoDriveBase.setBothMotors(0);

        telemetry.addData("LeftTaco Following Function","y = "+LeftSlope+"x+"+LeftIntercept);
        telemetry.addData("RightTaco Following Function","y = "+RightSlope+"x+"+RightIntercept);
        Waller = hardwareMap.opticalDistanceSensor.get("Waller");
        setStatus("Initialized");
        waitForStart();

//        setBothMotors(SearchingForWallSpeed);
//        setStatus("Searching for Wall");
//        while(opModeIsActive()){
//            SendOpsRaw();
//            updateTelemetry();
//            if(Waller.getRawLightDetected()>= TargetWallLightRaw) {
//                setBothMotors(0);
//                setStatus("Wall Found!");
//                updateTelemetry();
//                sleep(500);
//                break;
//            }
//            waitOneFullHardwareCycle();
//        }

        boolean followingWall = true;
        setStatus("Following Wall");
        while(followingWall && opModeIsActive()){
            followWall();
            SendOpsRaw();
            updateTelemetry();
            //To be replaced with some nice stopping logic, triggered by some king of sensor.  right now, we don't care.
            if(false){
                followingWall = false;
                break;
            }
        }
        tacoDriveBase.setBothMotors(0);
        setStatus("Stopped");
    }

    private void SendOpsRaw() {
        telemetry.addData("OPSRAW",Waller.getRawLightDetected());
    }

    private void setStatus(String value) {
        currentStatus = value;
    }

    private void updateTelemetry(){
        telemetry.addData("Status",currentStatus);
        telemetry.update();
    }


    //Updates the motors to adapt to the latest light input, and only once, in the attempt to follow a wall.:
    private void followWall() {
        double rawLight = Waller.getRawLightDetected();
        double LinearLight =Math.pow(rawLight,-0.5);
        //Linear Equations.  They actually have a purpose in real life!!!
//        LeftTaco = (-5/3)*LinearLight + 1;
//        RightTaco = (5/3)*LinearLight-0.5;
        tacoDriveBase.LeftTaco = LeftSlope*LinearLight+LeftIntercept;
        tacoDriveBase.RightTaco = RightSlope*LinearLight+RightIntercept;
        tacoDriveBase.updateMotors();
    }



}
