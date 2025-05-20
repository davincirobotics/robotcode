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


//@Autonomous(name="4 specimens omg 0_0", group="Linear Opmode")


public final class FourSpecimensMaybe extends LinearOpMode {
    public class Lift {
        private DcMotor rightLift;
        private DcMotor leftLift;

        public Lift(HardwareMap hardwareMap) {
            rightLift = hardwareMap.get(DcMotor.class, "Right Slide");
            leftLift = hardwareMap.get(DcMotor.class, "perp");

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        public Action liftUp() {
            return new LiftUp();
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
    }

    // Declare OpMode members.
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-12, 63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Lift lift = new Lift(hardwareMap);
        Pickup pickup = new Pickup(hardwareMap);

        //TrajectoryActionBuilder

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                //pickup.setup(),
                new ParallelAction(
                        pickup.retract(),
                        pickup.specimenToHang(),
                        lift.liftUp(),
                        drive.actionBuilder(beginPose)
                                .setTangent(Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(0, 26), Math.toRadians(-90))
                                .build()
                ),
                lift.hangSpecimen(),//hang 1st
                new ParallelAction(
                        pickup.prepareForSpecimen(),
                        lift.liftDown(),
                        drive.actionBuilder(new Pose2d(0, 26, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-36, 36), Math.toRadians(-180))

                                //.setTangent(Math.toRadians(180))
                                //.splineToConstantHeading(new Vector2d(-36, 24), Math.toRadians(-90))

                                //.strafeTo(new Vector2d(-36, 24))

                                .setTangent(Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-48, 6), Math.toRadians(-180))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-48, 50), Math.toRadians(90))

                                .setTangent(Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-48, 24), Math.toRadians(-90))
                                .setTangent(Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-54, 12), Math.toRadians(-180))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-54, 50), Math.toRadians(90))

                                //.setTangent(Math.toRadians(-90))
                                //.splineToConstantHeading(new Vector2d(-54, 24), Math.toRadians(-90))
                                //.setTangent(Math.toRadians(-90))
                                //.splineToConstantHeading(new Vector2d(-64, 12), Math.toRadians(-180))
                                //.setTangent(Math.toRadians(90))
                                //.splineToConstantHeading(new Vector2d(-64, 50), Math.toRadians(90))

                                .setTangent(Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-54, 40), Math.toRadians(-90))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-54, 55), Math.toRadians(-90))

                                .build()
                ),
                new ParallelAction(
                        pickup.specimenToHang(),//pickup 2nd
                        lift.liftUp(),
                        drive.actionBuilder(new Pose2d(-54, 65, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(-50))
                                .splineToConstantHeading(new Vector2d(10, -10), Math.toRadians(-90))
                                .build()
                ),
                new SleepAction(0.4),
                lift.hangSpecimen(), //hng 2nd
                new ParallelAction(
                        pickup.prepareForSpecimen(),
                        lift.liftDown(),
                        drive.actionBuilder(new Pose2d(10, -10, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-40, 88), Math.toRadians(90))
                                .build()
                ),
                new ParallelAction(
                        pickup.specimenToHang(),//pickup 3rd
                        lift.liftUp(),
                        drive.actionBuilder(new Pose2d(-40, 85, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(-50))
                                .splineToConstantHeading(new Vector2d(-2, 15), Math.toRadians(-90))
                                .build()
                ),
                lift.hangSpecimen(), //hang trois
                new ParallelAction(
                        pickup.prepareForSpecimen(),
                        lift.liftDown(),
                        drive.actionBuilder(new Pose2d(-2, 15, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-40, 88), Math.toRadians(90))
                                .build()
                ),
                new ParallelAction(
                        pickup.specimenToHang(),//pickup 4rd
                        lift.liftUp(),
                        drive.actionBuilder(new Pose2d(-40, 88, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(-50))
                                .splineToConstantHeading(new Vector2d(-2, 15), Math.toRadians(-90))
                                .build()
                ),
                lift.hangSpecimen(), //hang quatro
                new ParallelAction(
                        pickup.prepareForSpecimen(),
                        lift.liftDown(),
                        drive.actionBuilder(new Pose2d(-2, 15, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-40, 65), Math.toRadians(180))
                                .build()
                )
        ));
    }
}