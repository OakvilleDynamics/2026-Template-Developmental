Generate the full-code Unit Test procedure

Develop schema for autos and a test auto (for the unit test)

Work through managing what gets passed to smartdashboard when (dev, pid tuning, vision calibration, prematch, auto, during match, etc.) concerns that the current body fo "stuff' being sent to dashboard will cause bandwidth crashing when connected to competition fields.

test EVERY piece of this codebase (all written by Claude, none of it has had any testing or validation)

Develop and impliment an auto PID tuner (will need future-state PID tuning SOP complete to derive this from, currently PID tuning SOP is a 70% draft that needs peer-review)

Create an intake template mechanism (call it intakeMechanism to be consistent with shooter and elevator)
    Have this mechanism control to either duty cycle or velocity PID, force the intake to retract if over-the-bumper extending type when 'intake' button is not held down by default (mechanism protection measure)
    
    Will need ability to allow the intake to stay out in some cases after intake button is unpressed (i.e.: intake being "down"/outside the bumper enlarges hopper that is actively holding game piece(s) that would prevent the intake from retracting inside of frame perimeter)

    Also need ability to have a 'stop intake' external command passed in with a property to just stop the roller or stop the roller and retract the intake (or or or... 3rd option to retract the intake but keep the roller running)

    May also need to think through how it would handle a multi-game piece type game like 2025, 2023, 2021, 2019 or 2017 ?? 

Future swerve drivetrain SW changes (moved from docs/DRIVETRAIN_BRINGUP.md)
    Choose the motor a Nova steer drives. SwerveHardware.steerMotorConfig assumes a NOVA steer drives a Minion. Let the robot pick NEO or Minion, e.g. split SteerMotorType.NOVA into NOVA_MINION / NOVA_NEO. (TODO in SwerveHardware.)

    Consider wiring the absolute encoders to the Novas. The Nova has an absolute-encoder mode (EncoderType.ABS, setAbsOffset, setPositionAbs). It would steer directly from the absolute encoder with no startup seeding. Needs rewiring all four modules and confirming the Thrifty encoder works on the Nova's port.
        Code needed: NovaMotor always calls useEncoderType(EncoderType.INTERNAL), and neither MotorIOConfig nor IMotorIO can select a different feedback sensor. Add a vendor-neutral feedback setting to MotorIOConfig (e.g. internal vs. absolute, plus the sensor-to-mechanism ratio). With ABS feedback on the azimuth shaft, NovaMotor must not apply the 24:1 gear ratio to setpoints and readback, getVelocity() returns 0, wrapping uses setAbsoluteWrapping, and steer gains need re-tuning (error is in azimuth rotations, not motor rotations).

    Choose Xbox or flight sticks automatically. DriverStation.getJoystickIsXbox(0) isn't reliable when RobotContainer is built, because the Driver Station often isn't connected yet at boot. Detecting it later (e.g. at enable) means rebuilding the drive command and button bindings then. For now, switch DRIVER_CONTROLLER in RobotContainer and redeploy.

Quiet the remaining Nova config errors in sim mode. ConfigVerifier now skips in simulation (startup went from ~12 s / ~6,500 console lines to ~2 s / ~1,100 lines), but each NovaMotor constructor still writes its config (applyConfiguration) to a Nova that doesn't exist, and ThriftyLib logs ~16 "No acknowledgment" / "Failed to write config" errors per Nova (~64 total). Deferred: handle this with the ThriftyLib 2026.3.2 migration (likely in the 2027 branch). 2026.3.2 is a rewritten, largely undocumented API (com.thrifty.nova.Nova) that also adds motor sim support (com.thrifty.sim), so NovaMotor gets reworked then anyway. Stay on 2026.1.2 until then. (frc/lib/hardware/motor/NovaMotor.java)
