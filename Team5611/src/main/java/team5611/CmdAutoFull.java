/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team5611;

import com.qualcomm.robotcore.hardware.DcMotor;

import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoFull implements TrcRobot.RobotCommand
{
    private static final int DriveAndLowerTicks = 1550;

    private enum State
    {
        DO_DELAY,
        START_LOWER_FROM_LANDER,
        LOWER_FROM_LANDER,
        LOWER_AND_DRIVE_LANDER,
        DRIVE_FROM_LANDER,
        TURN_TO_DEPOT,
        DRIVE_TO_DEPOT,
        CLAIM_DEPOT,
        DEPOSIT_MARKER,
        RETRACT_FROM_DEPOT,
        DRIVE_FROM_DEPOT,
        EXTEND_ARM, DONE
    }   //enum State

    private static final String moduleName = "CmdTimedDrive";

    private Robot5611 robot;
    private double delay;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdAutoFull(Robot5611 robot, double delay) //This accepts the robot instance (which was created for us by FtcAuto) and the delay selected in the menus
    {
        this.robot = robot;
        this.delay = delay;
        event = new TrcEvent(moduleName); //'event' becomes a generic signaling device that can be reused between different states.
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdTimedDrive


    boolean firstIteration = false;
    State lastState = State.DONE;
    //
    // Implements the TrcRobot.AutoStrategy interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        if(state!=lastState){
            robot.tracer.traceInfo("cmdPeriodic","New State:  %s",state);
            firstIteration = true;
        }else{
            firstIteration = false;
        }
        lastState = state;
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");

        if (sm.isReady()) //This will be "no" if the state machine is disabled, or if it's waiting for an event.
        {
            switch (state)
            {
                case DO_DELAY:
                    State nextState = State.DRIVE_FROM_LANDER;
                    if (delay == 0.0)
                    {
                        sm.setState(nextState);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    break;
                case START_LOWER_FROM_LANDER: //STEP ONE:  START LOWERING THE LIFT
                    robot.roboLift(1); //You have to set the power to what you want it to move at, otherwise it won't move at all.
                    robot.roboLiftTicks(5400);
                    sm.setState(State.LOWER_FROM_LANDER); //Immediately set the next state to step two.  This step will not happen until the hardware has gone
                    // through a complete update cycle. (i.e., this function has completed and started again)

                    break;
                case LOWER_FROM_LANDER: //STEP TWO:  WAIT UNTIL THE ROBOT IS DONE LOWERING
                    if(robot.roboLiftOnTarget()){ //if the robot is done dropping
                        robot.roboLift(0); //stop the motor so it doesn't burn itself out
                        sm.setState(State.DRIVE_FROM_LANDER); //move on!
                    }
                    break;
                case LOWER_AND_DRIVE_LANDER:
                    if(firstIteration){
                        robot.roboLift(0.5);
                        robot.roboLiftTicks(DriveAndLowerTicks);
                        robot.tankDrive(0.2,0.2);
                    }else if(robot.roboLiftOnTarget()){
                        robot.roboLift(0);
                        sm.setState(State.DRIVE_FROM_LANDER);
                    }
                    break;
                case DRIVE_FROM_LANDER: //STEP THREE:  DRIVE A VERY SHORT WAYS
//                    robot.arcadeDrive(-0.25,0);
//                    robot.driveBase.arcadeDrive(-0.35,0); //'arcade drive" uses power and turn (between -1 and 1) to drive ('tank drive' uses left and right wheel powers)
//                    timer.set(0.5,event); //Fire this event after half a second)
//                    robot.encoderTankDrive(5,5,0.75,event);
                    event.clear();
                    robot.encoderTankDrive(6,6,0.5,event, false);
                    sm.waitForSingleEvent(event,State.TURN_TO_DEPOT); //When this event fires, move on to "Done"
                    break;
                case TURN_TO_DEPOT:
                    event.clear();
                    robot.encoderTankDrive(13.35,-13.35,0.5,event);
                    sm.waitForSingleEvent(event, State.EXTEND_ARM);
                    break;
                case EXTEND_ARM:
                    robot.extendoArm(1,1);
                    event.clear();
                    timer.set(2,event);
                    sm.waitForSingleEvent(event,State.DONE);
                    break;

//                case DRIVE_TO_DEPOT:
//                    event.clear();
//                    robot.encoderTankDrive(36,36,0.75,event);
//                    sm.waitForSingleEvent(event, State.CLAIM_DEPOT);
//                    break;
//                case CLAIM_DEPOT:
//                    event.clear();
//                    robot.extendoArm(1,-1);
//                    timer.set(5,event);
//                    sm.waitForSingleEvent(event, State.DONE);
//                    break;
                case RETRACT_FROM_DEPOT:
                    break;
                case DONE: //STEP FOUR:  DO NOTHING
                default:
                    //
                    // We are done.
                    //
                    robot.stopMotors();
                    robot.extendoArm(0,0);
                    done = true;
                    sm.stop(); //tell the state machine we're done.
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading()); //Spit some location and state info to the debug logs

            if (robot.pidDrive.isActive())
            {
                robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)", //Spit more info to the logs
                        robot.battery.getVoltage(), robot.battery.getLowestVoltage());
                robot.tracer.traceInfo("Raw Encoder",
                        "l=%.0f, r=%.0f",
                        robot.leftWheel.getPosition(),
                        robot.rightWheel.getPosition());

                robot.encoderYPidCtrl.printPidInfo(robot.tracer, elapsedTime);
                robot.gyroPidCtrl.printPidInfo(robot.tracer, elapsedTime);
            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdTimedDrive
