// Chapter 2: attitude dynamics (new). Uses the three schematics that the
// LaTeX sources drew but never included.
#import "../commands.typ": *
#import "../figures/schematics/coordinate_system.typ": coordinate-system
#import "../figures/schematics/virtual_rotation_ab.typ": virtual-rotation-ab
#import "../figures/schematics/tangential.typ": tangential

= Attitude dynamics <cha:attitude>
#context set-lhead[Chapter #counter(heading).get().first(). _Attitude dynamics_]

A quadrotor has no control surfaces: everything it does, it does by changing the speeds of its four rotors.
@cha:theoretical_foundations related rotor speed to thrust and torque; this chapter relates thrust and torque to the rotation of the vehicle.

== Coordinate frames <sec:frames>

The orientation, or _attitude_#index("attitude"), of the vehicle is described by the rotation of a body-fixed coordinate frame $(X, Y, Z)$ relative to the surroundings, with $Z$ pointing up along the rotor axes.
It is expressed by three angles, one about each axis of the body frame (see @fig:coordinate_system): the _roll_#index("attitude!roll") angle $Phi$ about $X$, the _pitch_#index("attitude!pitch") angle $Theta$ about $Y$ and the _yaw_#index("attitude!yaw") angle $Psi$ about $Z$.
Rotations do not commute, so the angles only describe an orientation together with the order in which they are applied; the order used by the controller has to be fixed when the equations of motion are set up.

#figure(
  placement: auto,
  {
    coordinate-system()
    caption-rule
  },
  caption: flex-caption(
    [Body frame and attitude angles],
    [Body-fixed coordinate frame with the attitude angles roll $Phi$, pitch $Theta$ and yaw $Psi$ about the $X$, $Y$ and $Z$ axes.],
  ),
) <fig:coordinate_system>

== Rotating the frame <sec:rotating_the_frame>

The four rotors $A$, $B$, $C$ and $D$ sit at the ends of two perpendicular arms of length $l$, measured from the center $R$ of the frame (see @fig:virtual_rotation_ab).
If all four produce the same thrust, the forces add up along $Z$ and the vehicle does not rotate.
If rotor $A$ produces more thrust than the opposite rotor $B$, the difference acts on the lever arm $l$ and rotates the frame about the axis through $C$ and $D$ by the angle $theta$, with a torque of

$ tau_(A B) = l (T_A - T_B) $ <eq:torque_ab>

and likewise $tau_(C D) = l (T_D - T_C)$ about the axis through $A$ and $B$.
With @eq:thrust_coefficient, each thrust grows with the square of its rotor speed, so that the torque about one axis is controlled by speeding up one rotor of a pair and slowing down the other.
If the sum of both changes is zero, the total thrust, and with it the altitude, stays approximately the same.

#figure(
  placement: auto,
  {
    virtual-rotation-ab()
    caption-rule
  },
  caption: flex-caption(
    [Rotation of the frame by rotors $A$ and $B$],
    [Quadrotor frame with center $R$, the rotors $A$ to $D$ and a coordinate frame per rotor. A thrust difference between $A$ and $B$ rotates the frame by $theta$ about the axis through $C$ and $D$.],
  ),
) <fig:virtual_rotation_ab>

Rotation about $Z$ uses a different effect.
Each rotor needs the driving torque $M$ of @eq:rotor_torque, and the frame feels the reaction to it, opposite to the direction of rotation.
The rotors $A$ and $B$ therefore spin in one direction and $C$ and $D$ in the other, so that the reaction torques cancel when all four turn at the same speed.
Speeding up $A$ and $B$ while slowing down $C$ and $D$ keeps the total thrust but leaves a net torque

$ tau_Z = (M_A + M_B) - (M_C + M_D) $ <eq:torque_yaw>

that yaws the vehicle (with the sign depending on the direction of rotation of $A$ and $B$).
Since the drag torque is much smaller than the lever arm times a thrust difference, the yaw axis responds more slowly than roll and pitch.

== Why four rotors <sec:why_four_rotors>

A multirotor has to control four quantities independently: the total thrust $T$ for the altitude, and the three torques $tau_(A B)$, $tau_(C D)$ and $tau_Z$ for roll, pitch and yaw.
How many rotors that takes follows from counting.

The reaction torque and the thrust of a rotor grow with the square of its speed alike, @eq:rotor_torque and @eq:thrust_coefficient, so their ratio is a constant length,

$ M = kappa T, quad kappa = (C_P D) / (2 pi C_T) $ <eq:torque_thrust_ratio>

about #qty("0.018", "m") for the propeller measured in @subsec:momentum_theory_to_motor, a tenth of a typical arm length; this is the number behind the weaker yaw axis above.
With it, all four quantities of the quadrotor are weighted sums of the four rotor thrusts:

$ vec(T, tau_(A B), tau_(C D), tau_Z) = mat(1, 1, 1, 1; l, -l, 0, 0; 0, 0, -l, l; kappa, kappa, -kappa, -kappa) vec(T_A, T_B, T_C, T_D) $ <eq:mixer>

The matrix is square and its rows are independent, so it can be inverted, and every combination of thrust and torques has exactly one set of rotor thrusts that produces it:

$ T_A &= T \/ 4 + tau_(A B) \/ (2 l) + tau_Z \/ (4 kappa), quad
  T_B = T \/ 4 - tau_(A B) \/ (2 l) + tau_Z \/ (4 kappa) \
  T_C &= T \/ 4 - tau_(C D) \/ (2 l) - tau_Z \/ (4 kappa), quad
  T_D = T \/ 4 + tau_(C D) \/ (2 l) - tau_Z \/ (4 kappa) $ <eq:mixer_inverse>

This inverse is the _mixer_#index("mixer") of a flight controller: the controllers compute the thrust and the three torques they want, and the mixer turns them into four motor commands.
Four rotors are thus the smallest number that gives four independent inputs without any moving part besides the rotors, and the symmetry makes roll and pitch identical.

With three fixed rotors, say on arms at 120° with rotor 1 on the $Y$ axis and rotors 1 and 2 spinning against rotor 3, the matrix of @eq:mixer has only three columns:

$ vec(T, tau_X, tau_Y, tau_Z) = mat(1, 1, 1; l, -l \/ 2, -l \/ 2; 0, -sqrt(3) l \/ 2, sqrt(3) l \/ 2; kappa, kappa, -kappa) vec(T_1, T_2, T_3) $ <eq:mixer_tri>

Three columns span at most three of the four directions, so one combination of thrust and torques cannot be produced at all.
Which one shows in hover: the two roll and pitch torques are only zero if all three thrusts are equal, and then the yaw torque is $kappa T \/ 3$, not zero.
A tricopter with fixed rotors cannot hover without spinning, which is the uncontrollable system of @subsec:controllability in its most practical form.
Real tricopters therefore add a fourth input, a servo that tilts the tail rotor sideways, so that a part of its thrust turns the vehicle about $Z$.
That works, but the servo is slower than the motors, tilting the rotor by an angle $delta$ also reduces its lift by the factor $cos delta$, so that yaw commands leak into pitch and altitude, and roll and pitch differ, since the three arms do not give them the same lever.
What a tricopter gains in return are fewer and, for the same footprint, larger propellers, which @subsec:area_vs_velocity showed to be more efficient.

With six or eight rotors, the matrix of @eq:mixer has more columns than rows, and every combination of thrust and torques can be produced by many sets of rotor thrusts.
The spare freedom is the reason to build them: it can be used to spread the load, or, depending on the arrangement, to keep flying after a motor fails.

== Angular acceleration <sec:angular_acceleration>

A torque $tau$ about an axis accelerates the frame according to

$ tau = I dot.op dot.double(alpha) $ <eq:angular_acceleration>

where $I$ is the moment of inertia about that axis and $dot.double(alpha)$ the angular acceleration.
A point at radius $r$ from the axis experiences the tangential acceleration $a_T = r dot.double(alpha)$ (see @fig:tangential), so the motors and rotors at the ends of the arms are the parts that have to be moved the most, and, being farthest from the axis, contribute the most moment of inertia per unit of mass.

#figure(
  placement: auto,
  {
    tangential()
    caption-rule
  },
  caption: flex-caption(
    [Tangential acceleration],
    [A point at radius $r$ rotated by the angle $alpha$ about the origin, with its tangential acceleration $arrow(a)_T$.],
  ),
) <fig:tangential>

How fast the attitude can change is thus limited twice: by how fast the motors can change the rotor speeds against the rotational inertia of the rotors (@subsec:area_vs_velocity), and by how fast the resulting torques can turn the frame against its own moment of inertia.
Both have to be known for the attitude controller of @cha:control, and @cha:identification shows how to measure them.
