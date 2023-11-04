package org.firstinspires.ftc.teamcode.Helper;


    /* Copyright (c) 2023 FIRST Tech Challenge - Team #404 “=ma” (https://...)
     *
     * Permission is granted, free of charge, to any person obtaining a copy of this software and
     * associated documentation (collectively, the "Software") to use the Software without restriction,
     * including without limitation, the rights to use, copy, modify, merge, publish, distribute,
     * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
     * furnished to do so; subject to the copyright notice above and this permission notice being
     * shall be included in all copies or substantial portions of the Software.
     *
     * THE SOFTWARE IS PROVIDED "AS IS"WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
     * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
     * NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY DIRECT,
     * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES OR OTHER LIABILITY,
     * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
     * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
     */


import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Date;

/*
     *  This class demonstrates the concepts for controlling a holonomic drivetrain using
     *  four(4) motors and GoBilda Mecanum wheels using human inputs from a gamepad.
     *
     *  Since all the wheels apply force at a 45% angle the motors in the drive train
     *  are configured in an X pattern this drivetrain supports omnidirectional movement.
     *
     *  This class collects telemetry data about its most recent values and exposes
     *  that data via (getTelemetry_...) methods.
     *
     *  More details about holonomic drivetrains can be found online:
     *     https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
     *
     */
    public class Drivetrain {

        //--------------------------------------------------------------
        //                        CONSTANTS
        // -------------------------------------------------------------
        //  TO DO: Move these constants to a configuration file

        // The strafing adjustment makes sliding movement (Left/Right) feel more natural as
        // the robot moves across the play field. this must be tuned for robot design and
        // driver preference.
        private static final float STRAFING_ADJUSTMENT = 1.08f; // Test

        // The Y axis on the joystick is reversed relative to the motor orientation
        private static final float JOYSTICK_Y_INPUT_ADJUSTMENT = -1f;

        private static final double BRAKING_STOP_THRESHOLD = 0.25;
        private static final double BRAKING_GAIN = 0.15; // Percent of Max Motor Power (1)
        private static final long BRAKING_INTERVAL = 100; // milliseconds
        private static final long BRAKING_MAXIMUM_TIME = (long) Math.ceil(1/BRAKING_GAIN) * BRAKING_INTERVAL;



        // Internal Class Variable
        private final DcMotor drvMotorFrontLeft;
        private final DcMotor drvMotorBackLeft;
        private final DcMotor drvMotorFrontRight;
        private final DcMotor drvMotorBackRight;
        private volatile boolean Braking_ON = false;


        // Telemetry Data
        private Date tlm_LastCalledTimestamp = new Date();
        private double tlm_LastPowerFrontLeft = 0f;
        private double tlm_LastPowerBackLeft = 0f;
        private double tlm_LastPowerFrontRight = 0f;
        private double tlm_LastPowerBackRight = 0f;
        private int tlm_BrakeCount = 0;
        private int tlm_BrakeTimeoutCount = 0;


        //--------------------------------------------------------------
        //                External Class Interface
        // -------------------------------------------------------------

        // Class Constructor
        public Drivetrain(@NonNull HardwareMap hdwMap) {
            drvMotorFrontLeft = hdwMap.dcMotor.get("frontLeft");
            drvMotorBackLeft = hdwMap.dcMotor.get("backLeft");
            drvMotorFrontRight = hdwMap.dcMotor.get("frontRight");
            drvMotorBackRight = hdwMap.dcMotor.get("backRight");

            // Account for motor mounting direction in our robot design
            drvMotorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        // Telemetry Data Getters
        public Date getTelemetry_LastCalledTimestamp() { return(tlm_LastCalledTimestamp); }
        public double getTelemetry_LastPowerFrontLeft() { return(tlm_LastPowerFrontLeft); }
        public double getTelemetry_LastPowerBackLeft() { return(tlm_LastPowerBackLeft); }
        public double getTelemetry_LastPowerFrontRight() { return(tlm_LastPowerFrontRight); }
        public double getTelemetry_LastPowerBackRight() { return(tlm_LastPowerBackRight); }
        public int getTelemetry_BrakeCount() { return(tlm_BrakeCount); }
        public int getTelemetry_BrakeTimeoutCount() { return(tlm_BrakeTimeoutCount); }



        /*
         * SetDriveVector : Computes drivetrain vector from three direction forces:
         *                      forward : (+) Forward   / (-) Backwards
         *                      strafe  : (+) Right     / (-) Left
         *                      rotate  : (+) clockwise / (-) counterclockwise
         *                  Then sets corresponding power of each motor in holonomic drivetrain.
         */
        public void SetDriveVector(double forward, double strafe, double rotate) {
            // Do Not Apply Power To The Motor While Braking;
            if (Braking_ON) return;

            // Denominator is the largest motor power (absolute value) or one (1)
            // This ensures all the motor powers maintain the same ratio
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

            double pwrFrontLeft = (forward + strafe + rotate) / denominator;
            double pwrBackLeft = (forward - strafe + rotate) / denominator;
            double pwrFrontRight = (forward - strafe - rotate) / denominator;
            double pwrBackRight = (forward + strafe - rotate) / denominator;

            // *** TO DO: Code to Handle Initial Motor Rampup from Stop (Power 0)
            // *** TO DO: Code to ramp large variations in Motor speed.
            drvMotorFrontLeft.setPower(pwrFrontLeft);
            drvMotorBackLeft.setPower(pwrBackLeft);
            drvMotorFrontRight.setPower(pwrFrontRight);
            drvMotorBackRight.setPower(pwrBackRight);

            // Store Telemetry Variables
            tlm_LastCalledTimestamp = new Date();
            tlm_LastPowerFrontLeft = pwrFrontLeft;
            tlm_LastPowerBackLeft = pwrBackLeft;
            tlm_LastPowerFrontRight = pwrFrontRight;
            tlm_LastPowerBackRight = pwrBackRight;

        }


        /*
         * SetHeadingFromJoystick : Uses two joystick inputs to compute force vectors for
         *                          each motor in the Holonomic drivetrain.
         */
        public void SetDriveVectorFromJoystick(float stickLeftX, float stickRightX, float stickY)  {
            // Do Not Apply Power To The Motor While Braking;
            if (Braking_ON) return;

            // Adjust Joystick inputs
            double forward = stickY * JOYSTICK_Y_INPUT_ADJUSTMENT;
            double strafe = stickLeftX * STRAFING_ADJUSTMENT;
            double rotate = stickRightX;

            SetDriveVector(forward, strafe, rotate);
        }

        /*
         * GetBrakeStatus : Returns Status of Braking
         */
        public boolean getBrakeStatus() {
            return( Braking_ON );
        }

        /*
         * SetBrakeStatus : An Coaster Braking Function that slow all drivetrain motor in Increments
         *                  with fixed interval delays (ms) in between until all motors stop.
         *
         *                    Braking: ON == Slows drivetrain to a stop.
         *                    Braking: OFF == Resets drivetrain motors to coasting mode (default)
         *                                    to enable normal driving.
         *
         *      IMPORTANT : This method is blocking - Control may not be retuned to the caller for up
         *                  two (2) seconds
         */
        public void setBrakeStatus(boolean braking ) throws InterruptedException {
            Braking_ON = braking;

            // Store Telemetry Variables
            tlm_LastCalledTimestamp = new Date();
            ++tlm_BrakeCount;

            // Configure Motor to Brake or Coast at Zero Power
            DcMotor.ZeroPowerBehavior newZeroBehavior;
            if (braking)
                newZeroBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
            else
                newZeroBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

            if (drvMotorFrontLeft.getZeroPowerBehavior() != newZeroBehavior)
                drvMotorFrontLeft.setZeroPowerBehavior(newZeroBehavior);
            if (drvMotorBackLeft.getZeroPowerBehavior() != newZeroBehavior)
                drvMotorBackLeft.setZeroPowerBehavior(newZeroBehavior);
            if (drvMotorFrontRight.getZeroPowerBehavior() != newZeroBehavior)
                drvMotorFrontRight.setZeroPowerBehavior(newZeroBehavior);
            if (drvMotorBackRight.getZeroPowerBehavior() != newZeroBehavior)
                drvMotorBackRight.setZeroPowerBehavior(newZeroBehavior);

            // Slow motor speed in Increments with interval delays (ms) until all motors stop
            if (braking) {
                boolean all_stop = false;
                boolean timer_expired = false;
                long ms_BrakeStart = System.currentTimeMillis();

                while (!all_stop && !timer_expired) {
                    boolean FL_stop = CoasterBrakeMotor(drvMotorFrontLeft);
                    boolean BL_stop = CoasterBrakeMotor(drvMotorBackLeft);
                    boolean FR_stop = CoasterBrakeMotor(drvMotorFrontRight);
                    boolean BR_stop = CoasterBrakeMotor(drvMotorBackRight);

                    all_stop = FL_stop && BL_stop && FR_stop && BR_stop;
                    timer_expired = (System.currentTimeMillis() >= (ms_BrakeStart + BRAKING_MAXIMUM_TIME));

                    if (!all_stop && !timer_expired)
                        sleep(BRAKING_INTERVAL);
                }

                if (timer_expired) ++tlm_BrakeTimeoutCount;
            }
        }


        //--------------------------------------------------------------
        //                Internal Helper Functions
        // -------------------------------------------------------------

        // Computer New Motor Power using Braking Gain% and minimum Power Threshold
        private boolean CoasterBrakeMotor( DcMotor motor ) {
            double curPower = motor.getPower();
            boolean stopped = (curPower == 0);

            if (!stopped) {
                double newPower = curPower - (Math.signum(curPower) * BRAKING_GAIN);
                if (Math.abs(newPower) < BRAKING_STOP_THRESHOLD) newPower = 0;
                motor.setPower(newPower);
            }

            return (stopped);
        }

    }

