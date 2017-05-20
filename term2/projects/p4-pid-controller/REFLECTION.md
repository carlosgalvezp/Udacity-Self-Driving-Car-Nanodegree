PID CONTROLLER - REFLECTION
===========================

In this document we reflect about the PID controller project, answering
to the corresponding points in the rubric.

1) **Describe the effect each of the P, I, D components had in your implementation.**

- The **proportional** component (P) is mainly in charge of directly
**reducing the cross-track error (CTE)** and steering the vehicle towards the
center of the lane.
However, this component alone makes the car **oscillate**. This makes sense since
the P component alone will **not** straighten the vehicle wheels when the
finally we reach CTE = 0; they will remain slightly turned in the direction
the vehicle was moving in previously.

- The **differential** component (D) helps stabilizing the P controller, by
taking into account the **derivative** of the error, effectively
**reducing the oscillations**. Indeed, when the CTE is being reduced thanks
to the P component, the D component notices this change and smoothly counteracts
the effects of P, so that when the CTE = 0 the wheels of the vehicle are
straight and no overshoot (or minimal) is produced.

- The **integral** component (I) helps **reducing constant error** which
cannot be removed via the P or D components. This happens especially
in **sharp curves**. Without the I component, the vehicle tends to stay
on the outer side of the curve, because the P component is not enough
to bring the vehicle back to the center - it's already spending effort
in keeping the vehicle in the lane. Adding a small integral term, the vehicle
turns **and** at the same time goes back to the center of the lane.


2) **Describe how the final hyperparameters were chosen.**

To choose the final hyperparameters we performed **manual tuning**, as follows:

- Start with kp = ki = kd = 0.0.

- Progressively increment the P component (kp), until the vehicle is able
to follow the curves and **starts to oscillate**. We started with a value
of 0.1 and settled in 0.2, incrementing in steps of 0.2.

- Keeping the same kp, we now tune the D component (kd), progressively
increasing it until **the vehicle stops oscillating**, and takes the curves
in a smooth way. We started with the same value as previous kp (0.2)
in steps of 0.1, and settled in 1.5.

- Finally, we analyze the performance of the vehicle in **sharp curves**
to tune the I component. One must be extra careful with this component
since it multiplies the integral of the error, which can be a big number.
If I is too big, the vehicle will start to oscillate again or go
completely off track. Therefore a **very small value must be used**.

We start with a value of 0.0001 and go progressively up until
**the vehicle is more centered in the sharp curves**. A final value of
0.001 was chosen, with steps of 0.0005.

This configuration allows the vehicle to complete the training track
at 30 km/h rather smoothly. There are still some oscillations that
could be improved. We tried to do this by further increasing the D component (kd),
but at some point the vehicle started to oscillate even more than before.

In general, we can observe that large constants (either kp, ki or kd) greatly
contribute to a very shaky vehicle motion, with extreme values for steering quite
often.


