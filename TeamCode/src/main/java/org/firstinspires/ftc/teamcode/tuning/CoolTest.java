package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.opencv.core.Mat;

//@Autonomous(name="cool test", group="Linear Opmode")

public final class CoolTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 49, Math.toRadians(-90));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            /*.setTangent(Math.toRadians(-180))
                            .splineToConstantHeading(new Vector2d(-48, 48), Math.toRadians(-180))
                            .setTangent(Math.toRadians(-90))
                            .splineToConstantHeading(new Vector2d(-48, -48), Math.toRadians(90))
                            .setTangent(Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(48, -48), Math.toRadians(0))
                            .setTangent(Math.toRadians(-90))
                            .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(-90))*/

                            .strafeTo(new Vector2d(-48, 48))
                            .strafeTo(new Vector2d(-48, -48))
                            .strafeTo(new Vector2d(48, -48))
                            .strafeTo(new Vector2d(48, 48))
                            .strafeTo(new Vector2d(0, 48))

                            .build());
        }  else if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
