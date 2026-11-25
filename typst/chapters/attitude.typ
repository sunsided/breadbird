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
