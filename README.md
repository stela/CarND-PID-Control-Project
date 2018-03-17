[![Build Status](https://travis-ci.org/stela/CarND-PID-Control-Project.svg?branch=master)](https://travis-ci.org/stela/CarND-PID-Control-Project)

# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Building

See build instructions in [the original README](README-original.md#basic-build-instructions). In addition, Travis-ci [![Build Status](https://travis-ci.org/stela/CarND-PID-Control-Project.svg?branch=master)](https://travis-ci.org/stela/CarND-PID-Control-Project) builds every commit to make sure that it at least compiles on Ubuntu Linux with both gcc and clang. I develop on macOS High Sierra using CLion so it should work on a Mac as well.


## PID algorithm implementation

The PID algorithm implementation follows the structure taught in the lessons. [PID::TotalError()](src/PID.cpp#L22-L25) calculates the error (control output). [PID::SteerValue()](src/PID.cpp#L27-L37) calculates the actual steering output based on the error, making sure the output is clamped between -1 and +1.

## Effect of P, I and D components
The P component makes sure the car steers towards the center of the street. While doing so, it easily results in oscillations and overshoot, worse the larger the P value.

The D component can counter the oscillations and overshoot, dampening the compensation when the error is reduced too quickly.

The I component counters long-term errors. Since the integrated error is accumulated over potentially a long duration, this value should not be too large or it will cause instability.


## PID component tuning.

Since I had no idea which magnitude to use or where to start, I noticed Wikipedia's article on PID controllers had a simple looking algorithm called [Zigler-Nichols method](https://en.wikipedia.org/wiki/PID_controller#Ziegler%E2%80%93Nichols_method). I used it and got
* Ku = 0.1
* Tu = 280

Putting these into the formulas on Wikipedia gave these results:

| Kp  | Ki  | Kd  |
| --- | --- | --- |
| 0.60 Ku = 0.06 | 1.2 Ku / Tu = 0.00043 | 3 Ku Tu / 40 = 1.2 |

That made the whole track on first attempt with throttle=0.3, but with wheels on or very close to the curb, too much to feel safe. It was a bit too "loose", not steering enough, at sharp curves, which means Kp and Ki could be increased a bit. Further manual tuning then resulted in:

| Kp  | Ki   | Kd   |
| ---:| ---: | ---: |
| 0.1 | 0.0008 | 3.0 |

Driving with constant throttle of 0.3 felt a bit boring, so instead I created a function [calc_throttle()](src/main.cpp#L60-L67) which gives throttle 0.2 when the car is off-center or steers left/right sharply, and otherwise gives plenty gas, 0.9, when the car is well centered and driving straight. This allows the car to reach speeds of 75 Mph on straight pieces of road, while driving slower and safer at curvy parts.

The behaviour of the PID regulator seemed to depend on how much debug output was written to the console, I think control could be more consistent if wall-clock was measured instead of using one time unit per simulator measurement received.

## Sample Video

Look at the [sample video](https://youtu.be/Z2Yu7rt9gNg) showing the car completing the track safely.
