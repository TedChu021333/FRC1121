// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
    private double MaxSpeed = 0.1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(Constants.OperatorConstants.kDriverControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LEDSubsystem leds = new LEDSubsystem(TunerConstants.kCANBus);

    /** 自動模式選單（會出現在 Shuffleboard / Elastic 的 "Auto Chooser"） */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Named Command 一定要在建立 auto 選單之前註冊，否則 PathPlanner 找不到
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("LED Test"); // 預設 auto（不用在 Dashboard 選）
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    //註冊給 PathPlanner 用的 LED 指令。
    private void registerNamedCommands() {
        registerLed("LED Off",            () -> leds.off());
        registerLed("LED Green",          () -> leds.solid(LEDSubsystem.GREEN));
        registerLed("LED Breathe Blue",   () -> leds.breathe(LEDSubsystem.BLUE, 60));
        registerLed("LED Strobe Blue",    () -> leds.strobe(LEDSubsystem.BLUE, 20));
    }

    /** 同時註冊成 Named Command 和 Event Trigger（各自用一份新的 Command，避免重複使用同一物件）。 */
    private void registerLed(String name, Supplier<Command> effect) {
        NamedCommands.registerCommand(name, leds.trigger(effect.get()));
        new EventTrigger(name).onTrue(leds.trigger(effect.get()));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        drivetrain.registerTelemetry(logger::telemeterize);

        // 進入手動 (teleop) 時，把自動模式留下的燈效關掉
        RobotModeTriggers.teleop().onTrue(leds.off());

        joystick.circle().toggleOnTrue(leds.breathe(LEDSubsystem.BLUE, 60));    // 呼吸
        joystick.cross().toggleOnTrue(leds.strobe(LEDSubsystem.BLUE, 20));  // 爆閃
        joystick.triangle().toggleOnTrue(leds.rainbow(60));                     // 彩虹
        joystick.square().toggleOnTrue(leds.larson(LEDSubsystem.BLUE, 60));   // 跑馬燈
    }

    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();
        // 印到 Driver Station 主控台，方便確認選到哪個 auto
        System.out.println("[Auto] 選到的自動模式: " + (auto == null ? "null" : auto.getName()));
        return auto;
    }
}
