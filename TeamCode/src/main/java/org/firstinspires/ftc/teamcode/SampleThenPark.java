package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.network.PasswordManagerFactory;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Mat;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;




//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;




import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Math;


@Autonomous(name="left sample park", group="Linear Opmode")
/*
for this one make sure these values are these:
        public double inPerTick = 0.0019758788;
        public double lateralInPerTick = 0.0013876365614925708;
        public double trackWidthTicks = 7267.9699580502665;

        // feedforward parameters (in tick units)
        public double kS =  0.8584586722606327;
        public double kV = 0.00039291891727863897;
        public double kA = 0.00008;

        // path profile parameters (in inches)
        public double maxWheelVel = 100;
        public double minProfileAccel = -75;
        public double maxProfileAccel = 75;

        // turn profile parameters (in radians)
        public double maxAngVel = (Math.PI)*1.75; // shared with path
        public double maxAngAccel = (Math.PI)*1.75;

        // path controller gains
        public double axialGain = 1.0;
        public double lateralGain = 10.0;
        public double headingGain = 10.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
 */

public final class SampleThenPark extends LinearOpMode {
    public class Lift {
        private DcMotor rightLift;
        private DcMotor leftLift;

        public Lift(HardwareMap hardwareMap) {
            rightLift = hardwareMap.get(DcMotor.class, "Right Slide");
            leftLift = hardwareMap.get(DcMotor.class, "perp");

            //rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            public boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightLift.setPower(1.0);
                    leftLift.setPower(1.0);
                    initialized = true;
                }

                double pos = rightLift.getCurrentPosition();
                packet.put("rightLiftPos", pos);
                //Position lift to high chamber (encoder=1200.0) for hanging speciman
                if (pos > -1000.0) {
                    return true;
                } else {
                    rightLift.setPower(0);
                    leftLift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftforLvl1 implements Action {
            public boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightLift.setPower(1.0);
                    leftLift.setPower(1.0);
                    initialized = true;
                }

                double pos = rightLift.getCurrentPosition();
                packet.put("rightLiftPos", pos);
                //Position lift to 2nd-bar (encoder=1200.0) for hanging speciman
                if (pos > -1000.0) {
                    return true;
                } else {
                    rightLift.setPower(0);
                    leftLift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftForLvl1() {
            return new LiftforLvl1();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightLift.setPower(-1.0);
                    leftLift.setPower(-1.0);
                    initialized = true;
                }

                double pos = rightLift.getCurrentPosition();
                packet.put("rightLiftPos", pos);
                if (pos < -5) {
                    return true;
                } else {
                    rightLift.setPower(0);
                    leftLift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }

        public class HangSpecimen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightLift.setPower(1.0);
                    leftLift.setPower(1.0);
                    initialized = true;
                }

                double pos = rightLift.getCurrentPosition();
                packet.put("rightLiftPos", pos);
                if (pos > -1850) {
                    return true;
                } else {
                    rightLift.setPower(0);
                    leftLift.setPower(0);
                    return false;
                }
            }
        }
        public Action hangSpecimen() {
            return new HangSpecimen();
        }

        public class LowBasket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightLift.setPower(1.0);
                    leftLift.setPower(1.0);
                    initialized = true;
                }

                double pos = rightLift.getCurrentPosition();
                packet.put("rightLiftPos", pos);
                if (pos > -1500) {
                    return true;
                } else {
                    rightLift.setPower(0);
                    leftLift.setPower(0);
                    return false;
                }
            }
        }
        public Action lowBasket() {
            return new LowBasket();
        }

        public class DownForLvl1 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightLift.setPower(-1.0);
                    leftLift.setPower(-1.0);
                    initialized = true;
                }

                double pos = rightLift.getCurrentPosition();
                packet.put("rightLiftPos", pos);
                if (pos < -1000) {
                    return true;
                } else {
                    rightLift.setPower(0);
                    leftLift.setPower(0);
                    return false;
                }
            }
        }
        public Action downForLvl1() {
            return new DownForLvl1();
        }
    }

    public class Pickup {
        private Servo topClaw;
        private Servo arm;
        private Servo topWrist;
        private Servo bottomClaw;
        private Servo bottomWrist;
        private Servo hSlide;

        public Pickup(HardwareMap hardwareMap) {
            topClaw = hardwareMap.get(Servo.class, "Top Claw");
            arm = hardwareMap.get(Servo.class, "Arm");
            topWrist = hardwareMap.get(Servo.class, "Top Arm");
            bottomWrist = hardwareMap.get(Servo.class, "Bottom Arm");
            bottomClaw = hardwareMap.get(Servo.class, "Bottom Claw");
            hSlide = hardwareMap.get(Servo.class, "Rack");
            //hSlide.setDirection(Servo.Direction.REVERSE);
            arm.setDirection(Servo.Direction.REVERSE);
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topClaw.setPosition(0);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topClaw.setPosition(0.5);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

        public class Setup implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPosition(0);
                topWrist.setPosition(0.6);
                hSlide.setPosition(0.01);
                return false;
            }
        }
        public Action setup() {
            return new Setup();
        }

        public class SpecimenToHang implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topClaw.setPosition(0);
                sleep(400);
                topWrist.setPosition(0);
                sleep(200);
                arm.setPosition(0.4);
                sleep(500);
                topWrist.setPosition(1.0);

                return false;
            }
        }
        public Action specimenToHang() {
            return new SpecimenToHang();
        }

        public class PrepareForSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topClaw.setPosition(0.2);
                sleep(300);
                topWrist.setPosition(0);
                arm.setPosition(1.0);
                sleep(1000);
                topWrist.setPosition(0.55);
                topClaw.setPosition(0.4);

                return false;
            }
        }
        public Action prepareForSpecimen() {
            return new PrepareForSpecimen();
        }

        public class ToGetSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //hSlide.setPosition(0.27);
                bottomClaw.setPosition(1.0);
                bottomWrist.setPosition(0.1);

                return false;
            }
        }
        public Action toGetSample() {return new ToGetSample();}

        public class Extend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlide.setPosition(0.27);

                return false;
            }
        }
        public Action extentd() {return new Extend();}
        public class ExtendLess implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlide.setPosition(0.13);

                return false;
            }
        }
        public Action extendLess() {return new ExtendLess();}

        public class PickUpSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bottomClaw.setPosition(0.0);
                bottomWrist.setPosition(0);
                sleep(500);
                bottomWrist.setPosition(0.1);

                return false;
            }
        }
        public Action pickupSample() {return new PickUpSample();}

        public class DropSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //hSlide.setPosition(0.1);
                //sleep(1000);
                bottomWrist.setPosition(0);
                bottomClaw.setPosition(1.0);
                sleep(500);
                hSlide.setPosition(0.01);

                return false;
            }
        }
        public Action dropSample() {return new DropSample();}

        public class HideLowClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bottomWrist.setPosition(0.8);
                bottomClaw.setPosition(1.0);

                return false;
            }
        }
        public Action hideLowClaw() {return new HideLowClaw();}

        public class RetractHS implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlide.setPosition(0.0);

                return false;
            }
        }
        public Action retract() {return new RetractHS();}

        public class LevelOneAScent implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topWrist.setPosition(0.65);
                arm.setPosition(0);
                topClaw.setPosition(0);

                return false;
            }
        }
        public Action stickArmOut() {return new LevelOneAScent();}

        public class DropLowBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topClaw.setPosition(0.3);

                return false;
            }
        }
        public Action dropLBasket() {return new DropLowBasket();}

        public class SampleForBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topClaw.setPosition(0.0);
                arm.setPosition(0);
                topWrist.setPosition(0.8);

                return false;
            }
        }
        public Action sampleForBasket() {return new SampleForBasket();}
    }

    // Declare OpMode members.
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(24, 63, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Lift lift = new Lift(hardwareMap);
        Pickup pickup = new Pickup(hardwareMap);

        //TrajectoryActionBuilder

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        pickup.retract(),
                        pickup.sampleForBasket(),
                        lift.lowBasket(),
                        drive.actionBuilder(beginPose)
                                .strafeTo(new Vector2d(60, 63))
                                .build()
                ),
                pickup.dropLBasket(),
                drive.actionBuilder(new Pose2d(60, 63, Math.toRadians(0)))
                        .strafeTo(new Vector2d(36, 63))
                        .build(),
                new ParallelAction(
                        pickup.sampleForBasket(),
                        drive.actionBuilder(new Pose2d(60, 63, Math.toRadians(0)))
                                //.strafeTo(new Vector2d(36, 63))
                                .turnTo(Math.toRadians(-90))
                                .strafeTo(new Vector2d(36, 12))
                                .turnTo(Math.toRadians(-180))
                                .strafeTo(new Vector2d(24, 12))
                                .build(),
                        lift.downForLvl1()
                )
        ));
    }
}
