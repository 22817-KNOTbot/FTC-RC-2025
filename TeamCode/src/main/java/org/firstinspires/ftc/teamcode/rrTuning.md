# RoadRunner Tuning
Note: Always update files **and** FTC Dashboard variables
## (Optional) Motor and Dead Wheel Direction
### found in MecanumDrive.java and ThreeDeadWheelLocalizer.java
> MecanumDirectionDebugger, DeadWheelDirectionDebugger
 - Self-explanatory
 - For dead wheels: increases forward and leftward
## Forward Push Test
### inPerTick found in MecanumDrive.java (and localizer uses)
> ForwardPushTest
 - Place on edge with measuring tape
 - Push slowly as far as possible
 - inPerTick = (double) inches/ticks
## Forward Ramp Logger
### kS and kV found in MecanumDrive.java
> ForwardRampLogger
 - Place on edge with room ahead
 - Start OpMode, stop as late as possible
 - [/tuning/forward-ramp.html](<http://192.168.43.1:8080/tuning/forward-ramp.html>)
 - Remove outliers (e and i keys)
## Lateral Ramp Logger
### lateralInPerTick found in MecanumDrive.java
> LateralRampLogger
 - Place on edge with room on left
 - Start OpMode, stop as late as possible
 - [/tuning/lateral-ramp.html](<http://192.168.43.1:8080/tuning/lateral-ramp.html>)
## Angular Ramp Logger
### trackWidthTicks found in MecanumDrive.java
### par0YTicks, par1YTicks, and perpXTicks found in ThreeDeadWheelLocalizer.java
> AngularRampLogger
 - Will spin increasingly fast
 - [/tuning/dead-wheel-angular-ramp.html](<http://192.168.43.1:8080/tuning/dead-wheel-angular-ramp.html>)
 - kS and kV required
 - Scroll down for more plots
## Manual Feedforward Tuner
### Refine kS, kV; add kA found in MecanumDrive.java
> ManualFeedforwardTuner
 - Drive forward/backward DISTANCE (default: 64)
 - Press Y for driver override. Press B to return
 - In Dashboard: graph `vref` against `v0`
 - Set kA to small value (0.0000001), then increase x10 until affects plot
 - Make 2 lines as close as possible
## Manual Feedback Tuner
### Tune feedback parameters
> ManualFeedbackTuner
 - Just read the [original](<https://rr.brott.dev/docs/v1-0/tuning/#manualfeedbacktuner>)
## Spline Test
### See if you're a failure or not
> SplineTest