# PID Step Response Tuner

An interactive browser tool for tuning a parallel PID controller on a mass-spring-damper plant
(m·ẍ + c·ẋ + k·x = u + F). Adjust the plant and PID parameters with sliders and see the step response
and metrics (rise time, overshoot, settling time, steady-state error, damping ratio).

The controller is u = Kp·e + Ki·∫e dt − Kd·dy/dt: the derivative acts on the output, not on the error,
so a textbook PID with the derivative on the error gives a somewhat different step response.

**Open the app:** https://vgroenhuis.github.io/pid-simulator/

Created for the Embedded Systems Laboratory course, University of Twente. The step response may not be
accurate; validate with a proper simulation program such as 20-sim or Matlab.

No build step: plain `index.html`, `style.css` and `pid.js`.
