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
