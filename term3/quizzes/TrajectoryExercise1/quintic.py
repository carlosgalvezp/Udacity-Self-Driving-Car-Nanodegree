import numpy as np

def JMT(start, end, T):
    """
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    """
    si    = start[0]
    si_d  = start[1]
    si_dd = start[2]

    sf    = end[0]
    sf_d  = end[1]
    sf_dd = end[2]

    a0 = si
    a1 = si_d
    a2 = si_dd * 0.5

    T_2 = T**2
    T_3 = T**3
    T_4 = T**4
    T_5 = T**5

    A = np.array([[      T_3,       T_4,        T_5],
                  [3.0 * T_2, 4.0 * T_3,  5.0 * T_4],
                  [6.0 * T,  12.0 * T_2, 20.0 * T_3]])

    b = np.array([[sf    - (si + si_d * T + 0.5 * si_dd * T_2)],
                  [sf_d  - (     si_d     +       si_dd * T  )],
                  [sf_dd - (                      si_dd      )]
                 ])

    x = np.dot(np.linalg.inv(A), b)

    a3 = x[0][0]
    a4 = x[1][0]
    a5 = x[2][0]

    return [a0,a1,a2,a3,a4,a5]
