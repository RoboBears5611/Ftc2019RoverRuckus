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
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoFull implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        LOWER_FROM_LANDER,
        DRIVE_FROM_LANDER,
        TURN_TOWARDS_VUFORIA,
        READ_VUFORIA_TARGET_FOR_SAMPLES,
        DRIVE_TO_SAMPLE_FIELD,
        FIND_GOLD_SAMPLE,
        SHOVE_GOLD_SAMPLE,
        BACK_UP_FOR_FINDING_VUFORIA,
        READ_VURFORIA_TARGET_FOR_FLAG,
        DRIVE_TO_DROP_ZONE,
        DEPOSIT_FLAG,
        START_LOWER_FROM_LANDER, DONE
    }   //enum State

    private static final String moduleName = "CmdTimedDrive";

    private Robot5611 robot;
    private double delay;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdAutoFull(Robot5611 robot, double delay)
    {
        this.robot = robot;
        this.delay = delay;
        event = new TrcEvent(moduleName);
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
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");

        if (sm.isReady())
        {
            state = sm.getState();

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.START_LOWER_FROM_LANDER); //NOTE:  MUST CHANGE BACK
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.START_LOWER_FROM_LANDER);
                    }
                    break;
                case START_LOWER_FROM_LANDER:
                    robot.roboLift(1);
                    robot.roboLiftTicks(5400);
                    sm.setState(State.LOWER_FROM_LANDER);

                    break;
                case LOWER_FROM_LANDER:
                    if(robot.roboLiftOnTarget()){
                        robot.roboLift(0);
                        sm.setState(State.DRIVE_FROM_LANDER);
                    }
                    break;
                case DRIVE_FROM_LANDER:
//                    robot.arcadeDrive(-0.25,0);
                    robot.driveBase.arcadeDrive(-0.35,0);
                    timer.set(0.5,event);
                    sm.waitForSingleEvent(event,State.DONE);
                    break;
                case TURN_TOWARDS_VUFORIA:
                    robot.leftWheel.motor.setTargetPosition(200);
                    if(!robot.leftWheel.motor.isBusy()){
                        sm.setState(State.DONE);
                    }
                    break;
                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.driveBase.arcadeDrive(0,0);
                    done = true;
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading());

            if (robot.pidDrive.isActive())
            {
                robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
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
