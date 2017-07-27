# PID Controller
Self-Driving Car Engineer Nanodegree Program - Elsa Wang

---

### Instruction

The code has compiled. It can run directly with terminal code `build/pid`.

### Files Change

- `PID.h` and `PID.cpp` at *src*.
  1. Complete `PID` for **steer angle**
  2. Add `throttle` function to control the **speed**
  3. Add `twiddle` functions to tune the hyperparameter
- `main.cpp`. Complete code to execute the simulation.
- `.mp4` files. Screen records for simulator running with ***30mph*** and ***50mph*** targeted speed and ***twiddle*** process.

### Reflection

#### Kp, Kd, Ki Components Effects

These three hyperparameters decide the performance of controller.

**Proportional Controller**

The core part of the PID, it changes steer angle based on the distance between current position to the target position(CTE). The parameter **Kp** mainly affects ***rise time*** and ***overshoot***. One decides how long the car gets to CTE = 0 from the beginning, and the other decides the oscillation. With **Kp** increasing, the **rise time** decreases sharply, but also more oscillation is introduced.

**Derivative Controller**

The vital part for cut down the oscillation. Derivative Controller decreases the overshoot by predicting the future movement of the car. Calculating difference between the current error and previous error, D Controller could minus the over steer angle before overshoot. More difference, less steer angle, and vice versa. By tuning its parameter Kd, D controller shows impressive performance on decreasing overshoot.

**Integral Controller**

Well, I think this is the most tricky part of PID. By accumulating the errors, it could increment controller ability to handle the noise, like random start position or tire beginning angles. However, it increases the overshoot as the same time. I tuned its parameter Ki carefully. The car performance didn't show an obviously increase, so tell the truth,  I think I doesn't play a vital role in this project.

#### Final Hyperparameters Chosen

I combined manual method and twiddle to tune the PID controller. Firstly, I tuned PID manually. I set all parameters to 0, then started to increase the Kd until it got `CTE = 0` with a relative high rise time(within 100 steps) and a relative less oscillation(don't over road too much). Then, I added to Pd steps by steps. Since the oscillation became small (about average CTE 0.5), I tried to add Ki. The last parameters got from manual methods was `Kp = 0.2, Kd = 2.0, Ki = 0`.

In account of manual parameters, I set start parameters for twiddle with  `Kp = 0.15, Kd = 2.0, Ki = 0`. Also, manual tuning let me know the sensitive unit of each parameters, so I set `dp = {0.015, .2, 0.00005}` for `Kp`, `Kd` and `Ki` separately.

After several iterations twiddle(really slow...), I got final hyperparameters `Kp = 0.1715, Ki = 0.0000156905, Kd = 2.98`.

I tried these hyperparameters with different target speeds. The highest speed that this PID could pass is 50mph, the average speed about 45mph.

The result shows that there're still spaces to improve this PID controller. To increase `Kd` seems to be a feasible way for the high speed. Beside the hyperparameters, to add a PID for `brake` maybe an approach to increase the stability for the car, especially for curving road.
