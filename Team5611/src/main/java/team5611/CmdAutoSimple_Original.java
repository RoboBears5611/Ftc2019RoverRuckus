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

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoSimple_Original implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        START_LOWER_FROM_LANDER,
        LOWER_FROM_LANDER,
        DRIVE_FROM_LANDER,
        CHUCK_TEAM_BALL,
        READ_VUFORIA_TARGET_FOR_SAMPLES, //These states were never used
        DRIVE_TO_SAMPLE_FIELD,
        FIND_GOLD_SAMPLE,
        SHOVE_GOLD_SAMPLE,
        BACK_UP_FOR_FINDING_VUFORIA,
        READ_VURFORIA_TARGET_FOR_FLAG,
        DRIVE_TO_DROP_ZONE,
        DEPOSIT_FLAG,
         DONE
    }   //enum State

    final static int DriveAndLowerTicks = 400;

    private static final String moduleName = "CmdTimedDrive";

    private Robot5611 robot;
    private double delay;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdAutoSimple_Original(Robot5611 robot, double delay) //This accepts the robot instance (which was created for us by FtcAuto) and the delay selected in the menus
    {
        this.robot = robot;
        this.delay = delay;
        event = new TrcEvent(moduleName); //'event' becomes a generic signaling device that can be reused between different states.
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdTimedDrive

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

        robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled"); //This function is explain in Robot5611

        if (sm.isReady()) //This will be "no" if the state machine is disabled, or if it's waiting for an event.
        {
            switch (state)
            {
                case DO_DELAY:
                    State nextState = State.START_LOWER_FROM_LANDER; //Define the next state once so you don't have to change it twice when you want it changed.
                    if (delay == 0.0)
                    {
                        sm.setState(nextState); //NOTE:  MUST CHANGE BACK
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    break;
                case START_LOWER_FROM_LANDER: //STEP ONE:  START LOWERING THE LIFT
                    robot.roboLift(1); //You have to set the power to what you want it to move at, otherwise it won't move at all.
                    robot.roboLiftTicks(5400 - DriveAndLowerTicks);
                    sm.setState(State.LOWER_FROM_LANDER); //Immediately set the next state to step two.  This step will not happen until the hardware has gone
                    // through a complete update cycle. (i.e., this function has completed and started again)

                    break;
                case LOWER_FROM_LANDER: //STEP TWO:  WAIT UNTIL THE ROBOT IS DONE LOWERING
                    if(robot.roboLiftOnTarget()){ //if the robot is done dropping
                        robot.roboLift(0); //stop the motor so it doesn't burn itself out
                        sm.setState(State.DRIVE_FROM_LANDER); //move on!
                    }
                    break;
                case DRIVE_FROM_LANDER: //STEP THREE:  DRIVE A VERY SHORT WAYS
//                    robot.arcadeDrive(-0.25,0);
                    robot.driveBase.arcadeDrive(-0.35,0); //'arcade drive" uses power and turn (between -1 and 1) to drive ('tank drive' uses left and right wheel powers)
                    timer.set(0.5,event); //Fire this event after half a second)
                    sm.waitForSingleEvent(event, State.DONE); //When this event fires, move on to "Done"
                    break;
                case CHUCK_TEAM_BALL:
                    robot.collector(1);
                    timer.set(1,event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;
                case DONE: //STEP FOUR:  DO NOTHING
                default:
                    //
                    // We are done.
                    //
                    robot.driveBase.arcadeDrive(0,0); //stop driving
                    done = true;
                    sm.stop(); //tell the state machine we're done.
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading()); //Spit some location and state info to the debug logs

            if (robot.vuforiaNavigator.pidDrive.isActive())
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
