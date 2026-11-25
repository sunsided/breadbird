// Section "Thrust" (latex/tex/chapters/theory/sections/thrust.tex).
#import "../../commands.typ": *
#import "../../figures/streamtube_schematic.typ": streamtube-schematic
#import "../../figures/propeller_thrust.typ": propeller-thrust
#import "../../figures/mass_flow_force.typ": mass-flow-force
#import "../../figures/climb_streamtube.typ": climb-streamtube
#import "../../figures/sensing_estimator.typ": sensing-estimator
#import "../../figures/mt_energy/mt_energy_3combined_linear.typ": mt-energy-3combined

/// \vpageref: "on this page", "on the next page", or "on page N".
#let vpageref(lbl) = context {
  let p = locate(lbl).page()
  let h = here().page()
  if p == h [on this page]
  else if p == h + 1 [on the next page]
  else if p == h - 1 [on the previous page]
  else [on page #link(locate(lbl), counter(page).display(at: locate(lbl)))]
}

== Thrust me, I'm an engine(er)

Thrust
is the propulsive force that strives to move a mass-accelerating system forwards.
It is a reaction force (in the sense of #smallcaps[Newton]'s _actio est reactio_)
that exists only in reaction to the acceleration of a mass, which happens to be a generalized fluid -- air -- in the case at hand.
Thrust is what makes propeller engines, jet propulsion engines, ion drives and garden
hoses
move forward.

In the next subsection,
the _actuator disk_ model shown in
@fig:streamtube_concept and @fig:actuator_disc #vpageref(<fig:streamtube_concept>)
will be used for the discussion of the propeller-based system:
Air from the _free stream_ at atmospheric pressure $p_0 = p_infinity$ and velocity $v_0$ (location #ncircled[0]) will be sucked into the propeller engine;
this is called the _inflow_.
While it approaches the rotor area at location #ncircled[1], air velocity increases towards $v_1 = v_i$ (the _induced velocity_#index("velocity!induced") or _downwash_#index("downwash") @seddon2002) while due to energy conservation, pressure decreases towards $p_1$.
Across the rotor disk (between #ncircled[1] and #ncircled[2]), pressure will be increased by an amount of $Delta p$ while air is promoted
to the _outflow_ at location #ncircled[2].
Since the rotor disk is assumed to be infinitesimally thin, however, air velocity does not change throughout the disk, so that $v_2 = v_1 = v_i$.
While velocity keeps rising towards $v_3$ between locations #ncircled[2] and #ncircled[3],
pressure falls again until it reaches atmospheric pressure $p_3 = p_infinity$ at location #ncircled[3].
Since energy and momentum of the rotating propeller is promoted to the air, this total increase in air velocity
$Delta v = v_3 - v_0$ -- due to the increase in pressure $Delta p$ -- essentially accounts for the resulting thrust.

#figure(
  placement: auto,
  {
    streamtube-schematic()
    caption-rule
  },
  caption: flex-caption(
    [Actuator disk concept of momentum theory],
    [Actuator disk concept of momentum theory: Visualization of the streamtube control volume,
      the actuator disk and the segments of conserved energy in the ranges of inflow (#ncircled[0]-#ncircled[1]) and outflow (#ncircled[2]-#ncircled[3]) by means of air pressure $p$ and velocity $v$. \ The distance #ncircled[1]-#ncircled[2] is
      assumed to be infinitesimally small.],
  ),
) <fig:streamtube_concept>

#figure(
  placement: auto,
  {
    propeller-thrust()
    caption-rule
  },
  caption: flex-caption(
    [Velocity and pressure in momentum theory],
    [Velocity and pressure in momentum theory: Stream tube geometry (left),
      change in air velocity ($Delta v$, dashed red) and
      air pressure relative to
      atmospheric pressure $p_infinity$ ($Delta p$, solid cyan).],
  ),
) <fig:actuator_disc>

Note that @fig:streamtube_concept relates a static actuator disk (of $arrow(v)_"disk" = 0$) to a nonzero free stream velocity $arrow(v)_0$.
This, by relativity of motion, is conceptually the same as relating a nonstatic actuator disk to a static free stream of velocity $arrow(v)_0 = 0$.
Because of this, $arrow(v)_0$ is used in the following to denote motion of the actuator disk itself within a static environment.


// MOMENTUM THEORY / STRAHLTHEORIE

=== Momentum theory in hover state <subsec:momentum_theory>

The air is assumed to be an incompressible fluid, i.e. no density difference will be experienced across the propeller area.
Under the principle of energy conservation and given the assumption of an incompressible mass flow, #name[Bernoulli]'s equation of pressure

$ p + rho g h + 1/2 rho v^2 = "const." $

may be applied to the inflow (#ncircled[0]-#ncircled[1]) and outflow (#ncircled[2]-#ncircled[3]) sections of the accelerating system @seddon2002.
Here, $p$, given in #withunit("N/m^2"), is the air pressure,
$rho$, in #withunit("kg/m^3"), its density,
$v$, in #withunit("m/s"), the flow velocity,
$g$, in #text(fill: darkgray, $tilde$)#withunitx("9.81", "m/s^2"), the gravitational acceleration and
$h$, in #withunit("m"), the height above a reference point.

The term $rho g h$ describes the hydrostatic pressure of the air column.
The undisturbed atmosphere is itself in hydrostatic equilibrium, i.e. its pressure already varies with height such that $p_infinity (h) + rho g h = "const."$; measuring all pressures relative to the ambient pressure at the same height therefore removes the term from the balance altogether.
The term is therefore omitted, such that

$ p + 1/2 rho v^2 = "const." $ <eq:bernoulli>

Applying @eq:bernoulli to the inflow section, energy conservation yields

$ p_infinity + 1/2 rho v_0^2 &= p_1 + 1/2 rho v_i^2 $ <eq:bernoulli_inflow>

using the equivalence $p_0 = p_infinity$; similarly, the outflow section gives

$ p_2 + 1/2 rho v_i^2 &= p_infinity + 1/2 rho v_3^2 $ <eq:bernoulli_outflow>

again using $p_3 = p_infinity$.
Expressing $p_2$ in terms of $p_1$ and a pressure difference $Delta p$ created by the rotor, such that

$ Delta p + p_1 &= p_2 $

results in

$ Delta p + (p_1 + 1/2 rho v_i^2) &= (p_2 + 1/2 rho v_i^2) $ <eq:bernoulli_induced_deltap_noident>

Substituting @eq:bernoulli_inflow and @eq:bernoulli_outflow into @eq:bernoulli_induced_deltap_noident then yields

$ Delta p + (p_infinity + 1/2 rho v_0^2) &= (p_infinity + 1/2 rho v_3^2) $ <eq:bernoulli_induced_deltap>

and thus#footnote[Using @eq:bernoulli_inflow and @eq:bernoulli_outflow, calculating
  $(p_2 - p_1) = (p_infinity + 1/2 rho v_3^2 - 1/2 rho v_i^2) - (p_infinity + 1/2 rho v_0^2 - 1/2 rho v_i^2)$ directly yields the same result.]

$ Delta p &= 1/2 rho (v_3^2 - v_0^2) $ <eq:pressure_difference_v3>

The actuator disk concept assumes that the volume swept out by the rotating propeller blades describes an infinitesimally thin disk (the so-called _rotor disk_) with orifice area $A = pi r^2$, where $r$ (in #withunit("m")) is the length of the propeller blades (i.e. the radius of the propeller).
Because of the disk being infinitesimally thin, the air velocity across the disk can be assumed to be continuous, i.e. the change in air velocity across the disk is zero.

Given the orifice area $A_r$ (the cross-section of the rotor disk), the volumetric flow rate
(or flow volume, given in #withunit("m^3/s"))#footnote[When
  thinking about the garden hose, the flow volume might be easier to understand when expressed in terms of #withunitx("1000", "L/s") rather than #withunitx("1", "m^3/s"), which technically is the same.]
is defined as the flow of air through the disk by

$ Q = dot(V) = A_r v_i $ <eq:flowvolume>

where $dot(V) = dif V \/ dif t$, as well as the mass flow rate (in #withunit("kg/s")) as

$ dot(m) = rho Q = rho A_r v_i $ <eq:massflow>

where the notation $dot(m)$ is used to denote mass over time, i.e. $dif m \/ dif t$.

Since mass enters and leaves the control volume only through upstream and downstream orifice areas $A_u = pi r_u^2$ and $A_d = pi r_d^2$ (compare @fig:streamtube_concept), continuity equation gives that

$ dot(m) = rho A_u v_0 = rho A_r v_i = rho A_d v_3 $ <eq:massflow_continuity>

Further, momentum and energy induced by the rotor must be conserved.
Since $dot(m)$ does not change, the velocity $v_3$ is a function of the energy induced by the rotor and can thus be used to describe the force acting on the outflow through the rotor.

#figure(
  placement: auto,
  {
    mass-flow-force()
    caption-rule
  },
  caption: flex-caption(
    [Mass flow rate and force],
    [Relation of the mass flow rate $dot(m)$ acting on a disk of area $A = pi r^2$ to the resulting force $arrow(F)$.],
  ),
) <fig:mass_flow_force>

In order to relate the mass flow rate $dot(m)$ with the force acting on the rotor (or vice versa), #name[Newton]'s second law of motion

$ F &= dif / (dif t) (m v) $ <eq:newtons_2nd>

may be used @book:siekmann2008.
If the velocity is assumed to be static (i.e. constant), such that $dif v \/ dif t = 0$, the mass must be varying over time, which is the very definition of the mass flow rate $dot(m) = dif m \/ dif t$ (compare @eq:massflow).
This then leads to

$ F &= (dif m) / (dif t) v = dot(m) v quad "if" quad v = "const." $ <eq:newtons_2nd_mdot>

which is displayed graphically in @fig:mass_flow_force.

As can be seen in @fig:actuator_disc, the total change in air velocity due to the rotor with respect to the upstream and downstream orifice areas is

$ Delta v &= v_3 - v_0 $

Consequently and due to the identity given in @eq:massflow_continuity, the resulting force is

$
  F &= dot(m) thin Delta v \
    &= dot(m) (v_3 - v_0) #<eq:rotor_force_deltav>
$

As was already shown in @eq:pressure_difference_v3, the pressure difference $Delta p$ accounts for this velocity change $Delta v$; since pressure is force per unit area and the pressure jump acts uniformly on the whole disk, the force is

$
  F &= A_r thin Delta p \
    &= A_r (p_2 - p_1) #<eq:rotor_force_deltap>
$

Equating @eq:rotor_force_deltav and @eq:rotor_force_deltap and using the identities from @eq:massflow_continuity gives

$
  Delta p &= dot(m) / A_r (v_3 - v_0) \
          &= rho v_i (v_3 - v_0)
$

With @eq:pressure_difference_v3, this leads to

$
  rho v_i (v_3 - v_0) &= 1/2 rho (v_3^2 - v_0^2) #<eq:froude_rankine_proof> \
                      &= 1/2 rho (v_3 + v_0) (v_3 - v_0) #<equate:revoke> \
                  v_i &= 1/2 (v_3 + v_0) #<eq:froude_rankine_theorem>
$

which is called the #name[Froude]-#name[Rankine] theorem @seddon2002 @book:priha2006: half of the total velocity increase takes place before the disk, the other half behind it.

Using @eq:froude_rankine_theorem in @eq:rotor_force_deltav relates the force acting on the air to the induced velocity $v_i$, such that

$
  F &= rho A_r v_i (v_3 - v_0) \
    &= 1/2 rho A_r (v_3 + v_0) (v_3 - v_0) #<equate:revoke> \
    &= 1/2 rho A_r (v_3^2 - v_0^2) #<eq:thrust_mdot>
$

With the thrust $T$ defined to be the reaction force to the action of moving mass, it consequently is given using @eq:thrust_mdot as

$ arrow(T) &= -arrow(F) $ <eq:thrust>

and points towards the inflow; along the flow direction, $T = dot(m) (v_0 - v_3)$, which is negative for $v_3 > v_0$.
In the following, $F$ and $T$ are used interchangeably for the magnitude.
This is the theoretical _dynamic thrust_#index("thrust!dynamic") produced by the rotor.

In case of a free stream velocity of $v_0 = 0$, by applying the #name[Froude]-#name[Rankine] theorem, @eq:thrust_mdot simplifies to the _static thrust_#index("thrust!static") of

$
  F_0 &= 1/2 rho A_r v_3^2 #<eq:static_thrust_Av> \
  arrow(T)_0 &= -arrow(F)_0
$

This is the maximum theoretically possible thrust the propeller can create while kept static, i.e. unmoved with regard to the free stream.
Viewed differently, it is equal to the force required to keep such a system static when it operates on full power.

What happens when the system moves faster than its own outflow, i.e. $v_0 > v_3$, is discussed in @subsec:efficiency.

Furthermore, the induced velocity of the static case -- the _hover velocity_#index("velocity!hover") $v_h$ -- can be trivially calculated from @eq:static_thrust_Av using $v_i = v_3 \/ 2$ (compare @eq:froude_rankine_theorem) as

$
  F_0 &= 1/2 rho A_r (2 v_i)^2
       = 2 rho A_r v_i^2 #<equate:revoke> \
  v_h &equiv v_(i_0) = sqrt(F_0 / (2 rho A_r)) #<eq:hover_velocity>
$

=== Momentum theory in axial climb and descent

==== Axial climb

In axial climb, by relativity of motion, the free stream approaches the rotor with the climb velocity $abs(arrow(v)_c)$, that is

$ v_c = v_0 $

Following from that, the thrust force $F_c$ is given as

$ F_c &= dot(m) (v_3 - v_c) $

which is analogous to @eq:rotor_force_deltav #vpageref(<eq:rotor_force_deltav>), with a velocity through the disk of

$ v_(i_c) &= 1/2 (v_3 + v_c) $

It is convenient to split this velocity into the climb velocity and the part the rotor adds itself, $v_(i_c) = v_c + u$.
By @eq:froude_rankine_theorem, the far wake then moves at $v_3 = v_c + 2 u$, just as the wake velocity in hover is twice the induced velocity (see @fig:climb_streamtube).

#figure(
  placement: auto,
  {
    climb-streamtube()
    caption-rule
  },
  caption: flex-caption(
    [Stream tube in axial climb],
    [Stream tube of a rotor climbing at $v_c$ (blue, relative to the still air), seen from the rotor: the air approaches with the climb velocity $v_c$, passes the disk at $v_c + u$ and leaves the far wake at $v_c + 2 u$, where $u$ is the velocity induced by the rotor.],
  ),
) <fig:climb_streamtube>

By the work--energy theorem, the work done on the air equals the change of its kinetic energy; the power $P_c$ the rotor has to deliver is therefore the kinetic energy it adds to the air per unit of time:

$
  P_c &= E(v_c + Delta v) - E(v_c) \
      &= 1/2 dot(m) (v_c + Delta v)^2 - 1/2 dot(m) (v_c)^2 \
      &= 1/2 dot(m) Delta v (2 v_c + Delta v)
$

with alternative forms being

$
  P_c &= 1/2 dot(m) (v_3^2 - v_c^2) \
      &= 1/4 rho A_r (v_3 + v_c) (v_3^2 - v_c^2) \
      &= 1/4 rho A_r (v_3 - v_c) (v_3 + v_c)^2 \
      &= rho A_r Delta v thin v_(i_c)^2 #<equate:revoke> \
      &= F_c thin v_(i_c) #<eq:climb_power>
$

The power is thus the thrust times the velocity of the air through the disk; for $v_c = 0$, it reduces to the ideal hover power $P_h = F v_h = sqrt(F^3 \/ (2 rho A_r))$.

If the rotor climbs with the same thrust it needs to hover, $F_c = 2 rho A_r (v_c + u) u$ must equal $F = 2 rho A_r v_h^2$, so that $(v_c + u) u = v_h^2$ and

$ u = -v_c / 2 + sqrt((v_c / 2)^2 + v_h^2) $ <eq:climb_inflow>

The induced velocity $u$ thus decreases as the climb velocity grows @book:priha2006.
Climbing still costs more power than hovering, $P_c = F (v_c + u) > F v_h$, but the extra power is less than $F v_c$, the rate at which the vehicle gains potential energy, because $u$ drops below $v_h$.

==== Axial descent <subsec:descent>

In descent, $v_c < 0$, and the free stream approaches the rotor from below, against the flow the rotor induces.
The stream tube of @fig:climb_streamtube only exists as long as the air passes the disk in a single direction, which holds for fast descent with $v_c <= -2 v_h$: the rotor then brakes the air like a windmill (the _windmill brake state_#index("windmill brake state")), and momentum theory yields $u = -v_c \/ 2 - sqrt((v_c \/ 2)^2 - v_h^2)$ @book:priha2006.
Between hover and that limit, $-2 v_h < v_c < 0$, the rotor sinks into its own wake; the flow recirculates around the blade tips (the _vortex ring state_#index("vortex ring state")), becomes unsteady, and momentum theory does not apply @book:priha2006.

=== Limits of momentum theory <subsec:limits>

The major flaw with these equations is that they assume the induced velocity $v_i$ to be constant across the actuator disk and that no air mass leaves the control volume, as otherwise the assumption of momentum conservation would not hold.
In reality, this will not be generally true, as, for example, the induced velocity varies due to the shape of the propeller blades, most notably so at the blade tips and, of course, the hub @book:priha2006.
For model rotors, @standschub lists the resulting losses: profile drag of the blades, which accounts for the largest share at up to 25 % of the power, the non-uniform induced velocity, swirl in the wake at no more than 5 %, and a loss of thrust at the blade tips.
In practice, these effects are captured by measured thrust and power coefficients, which are introduced in @subsec:momentum_theory_to_motor.

A more detailed model is _blade element theory_#index("blade element theory") (BET), which treats each radial slice of a blade as a two-dimensional airfoil and integrates its lift and drag over the radius.
Combined with momentum theory applied to concentric rings of the disk, it becomes _blade element momentum theory_ (BEMT), which yields the radial distribution of the induced velocity that momentum theory had to assume constant @book:priha2006.
Both are beyond the scope of this chapter.

// RELATION OF ROTOR SIZE TO OUTFLOW VELOCITY

=== Relation between propeller size and outflow velocity <subsec:area_vs_velocity>

As @eq:thrust_mdot and @eq:static_thrust_Av are directly dependent on the volumetric flow rate given in @eq:flowvolume and the outflow velocity $arrow(v)_3$,
it can be seen that there are different ways to obtain the same amount of (static) thrust: The system may either have

- a small orifice area $A$ with a high outflow velocity $arrow(v)_3$, or
- a large orifice area $A$ with a small outflow velocity $arrow(v)_3$.

More specifically, whenever the area $A$ quadruples, the velocity $arrow(v)_3$ may halve.
However, when relating the thrust with the power required to obtain it, the tradeoff between the two options is clear.

Generally, the equation for the kinetic energy#index("energy!kinetic") of a mass $m$ moving with velocity $v$ is given as

$ E &= 1/2 m v^2 $

and by the work--energy theorem, the power a rotor delivers is the kinetic energy it adds to the air per second (compare @eq:climb_power).
While -- with respect to the thrust -- it makes no difference to accelerate a mass flow of #qty("1", "kg/s") to #qty("10", "m/s") or a mass flow of #qty("10", "kg/s") to #qty("1", "m/s")

#[
  #set math.equation(numbering: none)
  $
    F_1 &= #qty("1", "kg/s") times #qty("10", "m/s") = #qty("10", "N") \
    F_2 &= #qty("10", "kg/s") times #qty("1", "m/s") = #qty("10", "N")
  $
]

the power differs by an order of magnitude:

#[
  #set math.equation(numbering: none)
  $
    P_1 &= 1/2 times #qty("1", "kg/s") times (#qty("10", "m/s"))^2 = #qty("50", "W") \
    P_2 &= 1/2 times #qty("10", "kg/s") times (#qty("1", "m/s"))^2 = #qty("5", "W")
  $
]

Since by energy conservation the energy added to the air must be taken from the rotor, it is much more efficient to use a larger propeller running at a lower speed (i.e. producing a lower induced velocity, yet creating a larger mass flow) than running a smaller propeller at a higher speed @book:mechflight2006.
In hover, the ideal power $P_h = sqrt(F^3 \/ (2 rho A_r))$ is inversely proportional to the rotor radius, so doubling the radius halves the power required for the same thrust.
@fig:v3_A_energy shows variations of outflow velocity $v_3$ and orifice area $A_r$ (in the range of $r_r = #qty("1", "cm") ... #qty("10", "cm")$) for different thrust amounts and the resulting effect on the power required.

#figure(
  placement: auto,
  {
    mt-energy-3combined()
    caption-rule
  },
  caption: flex-caption(
    [Outflow velocity and orifice area effects on power consumption],
    [Relation of orifice area $A_r$ and outflow velocity $v_3$ to the ideal power $P$ required for a fixed static thrust $F$ (with $rho = #qty("1.293", "kg/m^3")$).],
  ),
) <fig:v3_A_energy>

A larger propeller, however, is also heavier and carries its mass further out.
For geometrically similar blades, the mass grows with the cube of the radius, and the moment of inertia, $I tilde m r^2$, therefore with its fifth power.
Since a quadrotor controls its attitude solely by changing the speed of its rotors, a larger rotational inertia means that the motors need longer to change the thrust, which slows down the control response.

This is picked up again in @subsec:motor_matching, as it has an interesting impact on actuator design decisions, specifically in the choice of motor and propeller pairs.


// MAXIMUM PROPELLER EFFICIENCY / BETZ LIMIT

=== Maximum propeller efficiency <subsec:efficiency>

Momentum theory also bounds how efficiently a propeller can produce thrust.
For a propeller moving through the air at $v_0 > 0$, the useful power is the thrust times the flight speed, $F v_0$, while by @eq:climb_power the power delivered to the air is $F v_i$.
The ideal propulsive efficiency, or _Froude efficiency_#index("efficiency!Froude"), therefore is

$ eta_i = (F v_0) / (F v_i) = (2 v_0) / (v_0 + v_3) = 2 / (1 + v_3 \/ v_0) $ <eq:froude_efficiency>

and approaches unity only when the outflow velocity barely exceeds the flight speed, i.e. when a large mass flow is accelerated by a small amount; this is the same tradeoff as in @subsec:area_vs_velocity.
In hover, $v_0 = 0$ and no work is done on the vehicle, so that the efficiency is zero by definition.
Rotors are therefore rated by their _figure of merit_#index("figure of merit") $zeta$ instead, the ratio of the ideal power to the actual shaft power $P$ @book:priha2006 @standschub:

$ zeta = P_"id" / P = sqrt(F^3 \/ (2 rho A_r)) / P $ <eq:figure_of_merit>

For model rotors, Schenk gives values between about 0.5 and 0.7, with 0.55 to 0.6 for average to good rotors @standschub.

When the free stream is faster than the outflow, $v_0 > v_3$, the thrust from @eq:rotor_force_deltav changes sign and the rotor brakes the vehicle; the disk then extracts energy from the flow instead of adding it, i.e. it acts as a windmill.
For a fixed-pitch propeller, this happens in the vicinity of the _pitch speed_#index("velocity!pitch") $v_p = H n$, the distance $H$ the propeller would advance per revolution in a solid medium (like a wood screw, which air decidedly is not) times its rotational speed $n$: as the flight speed approaches $v_p$, the angle of attack of the blades, and with it the thrust, approaches zero.
Empirical thrust equations for model propellers, such as the one by Staples @staples2013, therefore take the pitch and the airspeed as inputs.

The limit for the windmill case is given by the #name[Lanchester]-#name[Betz]-#name[Joukowsky]#footnote[Although french spelling #name[Joukowsky] is used here, note that the transcription #name[Zhukovski] from the russian spelling is also commonly used @WE:WE218.] limit (discovered independently between 1915 and 1920, but generally attributed to #name[Betz] as the #name[Betz] limit @WE:WE218): an ideal actuator disk can extract at most $16 \/ 27 approx 59.3 thin %$ of the kinetic energy flowing through its area per unit of time.


// CALCULATION OF DYNAMIC THRUST USING C_t, C_p, K_t, Kp

=== Dynamic thrust

In comparison to @eq:thrust_mdot #vpageref(<eq:thrust_mdot>), an optimized equation for the _dynamic thrust_#index("thrust!dynamic") is given as

$ T_d = c_T 1/2 rho A_b v_b^2 $

where $v_b$ is the lateral blade velocity (e.g. tip velocity) and $A_b$ the blade area @physing[chapter 1.6.3].
This has the same form as the thrust coefficients of @subsec:momentum_theory_to_motor: the thrust is proportional to the air density, a reference area and the square of a blade velocity.
Blade element theory arrives at this form as well, with a coefficient that depends on the pitch, chord and number of the blades @book:priha2006.

// MOMENTUM THEORY TO MOTOR SPEED

=== Relating momentum theory to motor speed <subsec:momentum_theory_to_motor>

The quantities the flight controller actually commands are the rotational speeds of the motors.
The rotational speed $N$ is usually given in revolutions per minute (#unit("min^-1")) and relates to the rotational frequency $n$, the angular velocity $omega_R$ and the blade tip velocity $U$ of a rotor with radius $r$ as

$
  n &= N / 60 \
  omega_R &= 2 pi n = 2 pi N / 60 approx 0.105 dot N \
  U &= omega_R dot r
$

with $n$ in #unit("s^-1"), $omega_R$ in #unit("rad/s") and $U$ in #unit("m/s").
The torque $M$ (in #unit("N m")) driving the propeller relates to the shaft power $P$ (in #unit("W")) of the motor by

$ P = M dot omega_R $ <eq:shaft_power>

==== Thrust and power coefficients

Since the blades are airfoils, their lift and drag grow with the square of the airflow velocity, and with it the square of the rotational speed.
This is expressed by the dimensionless _thrust coefficient_#index("thrust!coefficient") $C_T$ and _power coefficient_#index("power coefficient") $C_P$, which in the convention commonly used for propellers are defined by @standschub

$
  T &= C_T dot rho n^2 D^4 #<eq:thrust_coefficient> \
  P &= C_P dot rho n^3 D^5 #<eq:power_coefficient>
$

with the propeller diameter $D = 2 r$ in #unit("m").
Other conventions use the tip velocity $U$ and a reference area instead, like the dynamic thrust equation above; for the same reference area, they differ only by constant factors @standschub.
Both coefficients are determined by measurements or numerical methods (such as BEMT) and hold for all geometrically similar propellers, as long as the Reynolds number does not change too much.

At a constant coefficient, the thrust grows with the square of the speed and the power with its cube: a speed increase of 10 % yields about 20 % more thrust and requires about 30 % more power.
At constant speed, the thrust grows with the fourth power of the diameter and the power with its fifth @standschub.

==== Static thrust from shaft power

Using the hover power from @eq:climb_power, the thrust an ideal rotor produces with a shaft power $P$ follows from $P = sqrt(F^3 \/ (2 rho A_r))$ as

$ F_"id" = root(3, 2 rho A_r P^2) $

and with the figure of merit from @eq:figure_of_merit, the static thrust#index("thrust!static") of a real rotor is @standschub

$ F_0 = root(3, 2 rho A_r (zeta P)^2) = zeta^(2 \/ 3) root(3, rho / 2 pi D^2 P^2) $ <eq:static_thrust_power>

using $A_r = pi D^2 \/ 4$.
A factor of 0.67 in front of the ideal formula, $F_0 approx 0.67 root(3, rho \/ 2 dot pi D^2 P^2)$, therefore corresponds to a figure of merit of $zeta approx 0.55$, since $0.55^(2 \/ 3) approx 0.67$.
Expressed by the coefficients of @eq:thrust_coefficient and @eq:power_coefficient, the figure of merit is

$ zeta = sqrt(2 / pi) dot C_T^(3 \/ 2) / C_P $

Schenk's measurements of an APC Slowfly 11×4.7 propeller ($D = #qty("0.277", "m")$, $A_r = #qty("0.0603", "m^2")$, $rho = #qty("1.24", "kg/m^3")$) serve as a check @standschub.
At $N = #qty("4264", "min^-1")$, it produced a thrust of #qty("4.01", "N") with a shaft power of #qty("31.6", "W").
An ideal rotor would have produced $F_"id" = root(3, 2 dot 1.24 dot 0.0603 dot 31.6^2) #unit("N") approx #qty("5.30", "N")$ with that power; by @eq:static_thrust_power, the measured thrust thus corresponds to a figure of merit of $zeta = (4.01 \/ 5.30)^(3 \/ 2) approx 0.66$, the value Schenk lists for this operating point.
From the same data, $C_T approx 0.109$ and $C_P approx 0.0435$.

==== Estimating the power coefficient

Without measurements, the power coefficient can be estimated from the pitch-to-diameter ratio $H \/ D$ of the propeller.
Schenk fitted the static power coefficients of a large number of model propellers by @standschub

$ C_P approx 0.090 dot H / D - 0.010 $

and for APC propellers alone by $C_P approx 0.0856 dot H \/ D - 0.0091$, with $H$ and $D$ in the same unit and $n$ in #unit("s^-1").
The fit describes the average over many propellers, though: for the Slowfly above ($H \/ D = 0.43$), the APC fit gives $C_P approx 0.027$, well below the measured 0.0435.

A different approach fits the measured power of each individual propeller with a law of the form $P = K_p dot (N \/ 1000)^(K_t)$, with $P$ in #unit("W") and two constants per propeller, a "propeller constant" $K_p$ and a "power factor" $K_t$; for an APC 6×4 propeller, $K_p = 0.015$ and $K_t = 3.2$, which yields about #qty("24", "W") at #qty("10000", "min^-1") @quadcopterproject.
For comparison, the APC fit above gives $C_P approx 0.048$ for this propeller ($H \/ D = 0.67$, $D = #qty("0.152", "m")$) and, with $rho = #qty("1.225", "kg/m^3")$, about #qty("22", "W").
Note that these $K_p$ are not power coefficients, and constants of the same name from different sources are generally not interchangeable @quadcopterproject.

==== Thrust-to-weight ratio

A vehicle can only take off vertically when its thrust exceeds its weight, i.e. when its _thrust-to-weight ratio_#index("thrust-to-weight ratio") $T \/ (m g)$ is greater than one.
The 17 t Hawker Siddeley Harrier, for example, accelerates vertically with the #qty("200", "kN") of its engine @wiki:schub, a thrust-to-weight ratio of about 1.2; a fixed-wing aircraft needs only a fraction of that, since its wings carry the rest of the weight.
To hover, the static thrust of a quadrotor's four rotors must equal its weight; to accelerate and to control its attitude, which means producing more thrust on one side than on the other, it needs a reserve above that.

=== Matching motor and propeller <subsec:motor_matching>

With @eq:shaft_power and @eq:power_coefficient, the propeller loads the motor with a torque of

$ M = P / omega_R = C_P / (2 pi) dot rho n^2 D^5 $

which grows with the square of the speed; the motor settles at the speed where its own torque equals this load.
For a brushless DC motor, the speed constant $K_v$ (in #unit("min^-1") per volt) states the no-load speed per volt of supply voltage, and its torque constant, the torque per ampere of current, is inversely proportional to $K_v$.
At a given battery voltage, a motor with a low $K_v$ therefore turns slower and delivers more torque for the same current, which suits a large propeller; a high-$K_v$ motor suits a small one.

Following @subsec:area_vs_velocity, the large propeller on a low-$K_v$ motor needs less power for the same thrust and is the choice for endurance, while the small propeller on a high-$K_v$ motor responds faster to speed changes, which favors agile flight.
How fast the rotors actually need to respond is a question for the attitude controller, and thus for a later chapter.

=== Sensing the airflow <subsec:sensing>

The flight controller commands rotor speeds, but what it needs are thrusts.
@eq:thrust_coefficient relates the two only as long as the air density and the inflow through the rotor stay the same; an updraft, the ground below the vehicle or the rotor's own wake change $C_T$ at constant speed.
The air flowing through the rotor cannot be measured directly, but three quantities that are easy to measure on the vehicle stand in for it: the rotor speed, the motor current and the pressure across the disk.

==== Rotor speed

A Hall sensor at the motor bell registers the passing magnet poles and thus the rotor speed $n$.
Sensorless brushless controllers derive the same information from the back EMF of the motor, so some controllers can report the speed without any additional sensor.

==== Motor current

The torque of a brushless DC motor is proportional to its current; with the torque constant $K_T = 60 \/ (2 pi K_v)$ (in #unit("N m/A") for $K_v$ in #unit("min^-1") per volt) and the no-load current $I_0$, which covers the motor's own friction losses, the shaft power follows with @eq:shaft_power as

$ P = K_T (I - I_0) dot omega_R $ <eq:power_from_current>

Together with the rotor speed, @eq:power_coefficient yields the load of the propeller,

$ rho C_P = P / (n^3 D^5) $ <eq:load_estimate>

and, with the air density $rho$ known from a barometer and a temperature sensor, the power coefficient itself.
The density changes only slowly, by about 1 % per 100 m of altitude; a change of $C_P$ at constant speed therefore indicates a change of the inflow rather than of the air.

==== Pressure across the disk

The pressure jump across the disk gives the thrust directly, by @eq:rotor_force_deltap:

$ T = A_r Delta p = A_r (p_2 - p_1) $

with $p_1$ measured just above and $p_2$ just below the disk.
Measuring both sides matters: in hover, the static pressure just below the disk exceeds the ambient pressure by only $p_2 - p_infinity = 3/2 rho v_i^2 = 3/4 Delta p$ (from @eq:bernoulli_inflow with $v_0 = 0$ and $v_3 = 2 v_i$), and this fraction changes with the inflow.

The signals are small: a rotor of #qty("10", "cm") radius producing #qty("1", "N") of thrust has a pressure jump of only $Delta p = #qty("1", "N") \/ (pi dot (#qty("0.1", "m"))^2) approx #qty("32", "Pa")$.
A differential pressure sensor with a range of a few hundred pascals suits this better than the difference of two absolute barometers.
Each passing blade adds a pressure pulse, at twice the rotational frequency for a two-bladed propeller, so the readings have to be averaged over several revolutions.
Since the induced velocity is not uniform (@subsec:limits), the ports should sit at a fixed radius, flush with the flow, and the measured $Delta p$ needs a calibration against a thrust stand.

==== Combining the measurements

@fig:sensing_estimator shows how the measurements combine.
With thrust, power, speed and density, the coefficients of @eq:thrust_coefficient and @eq:power_coefficient and the figure of merit of @eq:figure_of_merit can be tracked during flight.

#figure(
  placement: auto,
  {
    sensing-estimator()
    caption-rule
  },
  caption: flex-caption(
    [Estimating the rotor state from motor and pressure measurements],
    [Estimating the rotor state: speed, current and pressure across the disk yield rotor speed, shaft power and thrust, from which the coefficients, the figure of merit and the velocity through the disk follow.],
  ),
) <fig:sensing_estimator>

The measurements also give the velocity of the air through the disk.
By @eq:climb_power, the ideal power is $T (v_c + u)$; assuming the figure of merit $zeta_0$ measured in hover on a thrust stand also holds in flight, the actual power is $P approx T (v_c + u) \/ zeta_0$, and with @eq:climb_inflow

$
  w &= v_c + u approx zeta_0 P / T \
  u &= T / (2 rho A_r w) \
  v_c &= w - u
$

yields the axial velocity $v_c$ of the air relative to the rotor.
This estimate is sensitive to the assumed figure of merit: in hover, $v_c = w - v_h^2 \/ w$ changes twice as fast as $w$, so for $T = #qty("2", "N")$, $r = #qty("10", "cm")$ and $rho = #qty("1.225", "kg/m^3")$ ($v_h approx #qty("5.1", "m/s")$), an error of 10 % in $zeta_0$ already shifts $v_c$ by about #qty("1", "m/s").

Even without a reliable airspeed, the measurements show when the conditions change, since the rotor speed stays constant while current and pressure move:

- An updraft or downdraft through the rotor changes the angle of attack of the blades; thrust, pressure jump and current change together while the speed stays the same.
- Close to the ground, the rotor needs less power for the same thrust (_ground effect_#index("ground effect")), so that the apparent figure of merit rises @book:priha2006.
- In the vortex ring state (@subsec:descent), the thrust fluctuates and the power required rises @book:priha2006; a fluctuating pressure jump at constant speed is an early sign of it.

A thrust stand that records speed, current and thrust for each propeller comes first, as it provides $C_T$, $C_P$, $zeta_0$ and the pressure calibration that all of the above relies on.
