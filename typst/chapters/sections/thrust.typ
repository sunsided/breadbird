// Section "Thrust" (latex/tex/chapters/theory/sections/thrust.tex).
#import "../../commands.typ": *
#import "../../figures/streamtube_schematic.typ": streamtube-schematic
#import "../../figures/propeller_thrust.typ": propeller-thrust
#import "../../figures/mass_flow_force.typ": mass-flow-force
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

#todo[static thrust vs. jet velocity (pitch speed?)]

== Thrust me, I'm an engine(er)

Thrust
is the propulsive force that strives to move a mass-accelerating system forwards.
It is a reaction force (in the sense of #smallcaps[Newton]'s _actio est reactio_)
that exists only in reaction to the acceleration of a mass, which happens to be a generalized fluid -- air -- in our case.
Thrust is what makes propeller engines, jet propulsion engines, ion drives and garden
hoses
move forward.

In the next subsection,
the _actuator disk_ model shown in
@fig:streamtube_concept and @fig:actuator_disc #vpageref(<fig:streamtube_concept>)
will be used for the discussion of our propeller-based system:
Air from the _free stream_ at atmospheric pressure $p_0 = p_infinity$ and velocity $v_0$ (location #ncircled[0]) will be sucked into our propeller engine;
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
Because of this, we will use $arrow(v)_0$ in the following to denote motion of the actuator disk itself within a static environment.


// MOMENTUM THEORY / STRAHLTHEORIE

=== Momentum theory in hover state <subsec:momentum_theory>

We will assume the air to be an incompressible fluid, i.e. no density difference will be experienced across the propeller area.
Under the principle of energy conservation and given the assumption of an incompressible mass flow, #name[Bernoulli]'s equation of pressure

$ p + rho g h + 1/2 rho v^2 = "const." $

may be applied to the inflow (#ncircled[0]-#ncircled[1]) and outflow (#ncircled[2]-#ncircled[3]) sections of the accelerating system @seddon2002.
Here, $p$, given in #withunit("N/m^2"), is the air pressure,
$rho$, in #withunit("kg/m^3"), its density,
$v$, in #withunit("m/s"), the flow velocity,
$g$, in #text(fill: darkgray, $tilde$)#withunitx("9.81", "m/s^2"), the gravitational acceleration and
$h$, in #withunit("m"), the height above a reference point.

#todo[why may we ignore the $rho g h$ term? negligible compared to rotor pressure?]

Since for our purposes the term $rho g h$ can be assumed to be negligible, it may be ommited, such that

$ p + 1/2 rho v^2 = "const." $ <eq:bernoulli>

If we apply @eq:bernoulli to the inflow section, due to energy conservation we obtain

$ p_infinity + 1/2 rho v_0^2 &= p_1 + 1/2 rho v_i^2 $ <eq:bernoulli_inflow>

where we use the equivalence $p_0 = p_infinity$, similarly for the outflow section, we get

$ p_2 + 1/2 rho v_i^2 &= p_infinity + 1/2 rho v_3^2 $ <eq:bernoulli_outflow>

again using $p_3 = p_infinity$.
If we now express $p_2$ in terms of $p_1$ and a pressure difference $Delta p$ created by the rotor, such that

$ Delta p + p_1 &= p_2 $

we find that

$ Delta p + (p_1 + 1/2 rho v_i^2) &= (p_2 + 1/2 rho v_i^2) $ <eq:bernoulli_induced_deltap_noident>

By identity using @eq:bernoulli_inflow and @eq:bernoulli_outflow in @eq:bernoulli_induced_deltap_noident we obtain

$ Delta p + (p_infinity + 1/2 rho v_0^2) &= (p_infinity + 1/2 rho v_3^2) $ <eq:bernoulli_induced_deltap>

and thus#footnote[Using @eq:bernoulli_inflow and @eq:bernoulli_outflow, calculating
  $(p_2 - p_1) = (p_infinity + 1/2 rho v_3^2 - 1/2 rho v_i^2) - (p_infinity + 1/2 rho v_0^2 - 1/2 rho v_i^2)$ directly yields the same result.]

$ Delta p &= 1/2 rho (v_3^2 - v_0^2) $ <eq:pressure_difference_v3>

Using the actuator disk concept, we assume that the volume swept out by the rotating propeller blades describes an infinitesimally thin disk (the so-called _rotor disk_) with orifice area $A = pi r^2$, where $r$ (in #withunit("m")) is the length of the propeller blades (i.e. the radius of the propeller).
Because of the disk being infinitesimally thin, the air velocity across the disk can be assumed to be continuous, i.e. the change in air velocity across the disk is zero.

Given the orifice area $A_r$ (the cross-section of the rotor disk), we can then define the volumetric flow rate
(or flow volume, given in #withunit("m^3/s"))#footnote[When
  thinking about the garden hose, the flow volume might be easier to understand when expressed in terms of #withunitx("1000", "L/s") rather than #withunitx("1", "m^3/s"), which technically is the same.]
as the flow of air mass through the disk by

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
    [Relation of the mass flow rate $dot(m)$ acting on a disk of area $A = 2 pi r$ to the resulting force $arrow(F)$.],
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

As was already shown in @eq:pressure_difference_v3, the pressure difference $Delta p$ accounts for this velocity change $Delta v$, so that#todo(inline: false)[which law is that?]

$
  F &= A_r thin Delta p \
    &= A_r (p_2 - p_1) #<eq:rotor_force_deltap>
$

By equating @eq:rotor_force_deltav and @eq:rotor_force_deltap and using the identities from @eq:massflow_continuity we find

$
  Delta p &= dot(m) / A_r (v_3 - v_0) \
          &= rho v_i (v_3 - v_0)
$

With @eq:pressure_difference_v3 we then obtain

$
  rho v_i (v_3 - v_0) &= 1/2 rho (v_3^2 - v_0^2) #<eq:froude_rankine_proof> \
                      &= 1/2 rho (v_3 + v_0) (v_3 - v_0) #<equate:revoke> \
                  v_i &= 1/2 (v_3 + v_0) #<eq:froude_rankine_theorem>
$

which is called the #name[Froude]-#name[Rankine] theorem#todo(inline: false)[source].

Using @eq:froude_rankine_theorem in @eq:rotor_force_deltav we can now relate the force acting on the air to the induced velocity $v_i$, such that

$
  F &= rho A_r v_i (v_3 - v_0) \
    &= 1/2 rho A_r (v_3 + v_0) (v_3 - v_0) #<equate:revoke> \
    &= 1/2 rho A_r (v_3^2 - v_0^2) #<eq:thrust_mdot>
$

Since we have the thrust $T$ defined to be the reaction force to the action of moving mass, it consequently is given using @eq:thrust_mdot as#todo(inline: false)[it's easier just to $(v_0 - v_3)$ instead]

$ arrow(T) &= -arrow(F) $ <eq:thrust>

and points towards the inflow.
This is the theoretical _dynamic thrust_#index("thrust!dynamic") produced by the rotor.

In case of a free stream velocity of $v_0 = 0$, by applying the #name[Froude]-#name[Rankine] theorem, @eq:thrust_mdot simplifies to the _static thrust_#index("thrust!static") of

$
  F_0 &= 1/2 rho A_r v_3^2 #<eq:static_thrust_Av> \
  arrow(T)_0 &= -arrow(F)_0
$

This is the maximum theoretically possible thrust the propeller can create while kept static, i.e. unmoved with regard to the free stream.
Viewed differently, it is equal to the force required to keep such a system static when it operates on full power.

#todo[now if the system is moving faster than its jet velocity $v_"out"$, does that mean the thrust is negative and thus makes the system slower?]

Furthermore, the induced velocity of the static case -- the _hover velocity_#index("velocity!hover") $v_h$ -- can be trivially calculated from @eq:static_thrust_Av using $v_i = v_3 \/ 2$ (compare @eq:froude_rankine_theorem) as

$
  F_0 &= 1/2 rho A_r (2 v_i)^2
       = 2 rho A_r v_i^2 #<equate:revoke> \
  v_h &equiv v_(i_0) = sqrt(F / (2 rho A_r))
$

// SOMETHING SOMETHING

=== Momentum theory in axial climb and descent

==== Axial climb

#todo[add fig. 3.17 from @book:priha2006]

In axial climb, by relativity of motion, the atmospheric velocity is equal to the amount of the climb velocity $abs(arrow(v)_c)$, that is

$ v_c = v_0 $

Following from that, the thrust force $F_c$ is given as

$ F_c &= dot(m) (v_3 - v_c) $

which is analog to @eq:rotor_force_deltav #vpageref(<eq:rotor_force_deltav>), with an induced velocity of

$ v_(i_c) &= 1/2 (v_3 + v_c) $

Given the work-energy principle we know that the work done by the rotor must equal the energy change in the air, that is

$
  W_c &= E(v_c + Delta v) - E(v_c) \
      &= 1/2 dot(m) (v_c + Delta v)^2 - 1/2 dot(m) (v_c)^2 \
      &= 1/2 dot(m) Delta v (2 v_c + Delta v)
$
with alternative solutions being
$
  W_c &= 1/4 dot(m) (v_3^2 - v_c^2) \
      &= 1/4 rho A_r (v_3 + v_c) (v_3^2 - v_c^2) \
      &= 1/4 rho A_r (v_3 - v_c) (v_3 + v_c)^2 \
      &= 1/2 rho A_r Delta v thin v_i^2
$

// SOMETHING SOMETHING

=== The rest

The major flaw with these equations is that they assume the induced velocity $v_i$ to be constant across the actuator disk and that no air mass leaves the control volume, as otherwise the assumption of momentum conservation would not hold.
In reality, this will not be generally true, as, for example, the induced velocity varies due to the shape of the propeller blades, most notably so at the blade tips and, of course, the hub#todo(inline: false)[source].
In order to model these effects, the thrust coefficient $c_T$ will be introduced later.

#todo[where is it introduced?]

#todo[add hint towards BET or BEMT]

// RELATION OF ROTOR SIZE TO OUTFLOW VELOCITY

=== Relation between propeller size and outflow velocity <subsec:area_vs_velocity>

As @eq:thrust_mdot and @eq:static_thrust_Av are directly dependent on the volumetric flow rate given in @eq:flowvolume and the outflow velocity $arrow(v)_3$,
it can be seen that there are different ways to obtain the same amount of (static) thrust: The system may either have

- a small orifice area $A$ with a high outflow velocity $arrow(v)_3$, or
- a large orifice area $A$ with a small outflow velocity $arrow(v)_3$.

More specifically, whenever the area $A$ quadruples, the velocity $arrow(v)_3$ may halve.
However, when relating the thrust with the energy required to obtain it, the tradeoff between the two options is clear.

Generally, the equation for the kinetic energy#index("energy!kinetic") of a mass $m$ moving with velocity $v$ is given as

$ E &= 1/2 m v^2 $

#todo[relate this to the work-energy theorem #link("http://en.wikipedia.org/wiki/Work_%28physics%29#General_derivation_of_the_work-energy_theorem_for_a_particle")]
While -- with respect to the thrust -- it would make no difference to accelerate a mass of #qty("1", "kg") to #qty("10", "m/s") or a mass of #qty("10", "kg") to #qty("1", "m/s")

#[
  #set math.equation(numbering: none)
  $
    F_1 &= #qty("1", "kg") times #qty("10", "m/s^2") = #qty("10", "N") \
    F_2 &= #qty("10", "kg") times #qty("1", "m/s^2") = #qty("10", "N")
  $
]

the difference in energy is of an order of magnitude

#[
  #set math.equation(numbering: none)
  $
    E_1 &= 1/2 times #qty("1", "kg") times (#qty("10", "m/s"))^2 = #qty("50", "J") \
    E_2 &= 1/2 times #qty("10", "kg") times (#qty("1", "m/s"))^2 = #qty("5", "J")
  $
]

Since by energy conservation the energy added to the air must be taken from the rotor, it is much more efficient to use a larger propeller running at a lower speed (i.e. producing a lower induced velocity, yet creating a larger mass flow) than running a smaller propeller at a higher speed @book:mechflight2006.
@fig:v3_A_energy shows variations of outflow velocity $v_3$ and orifice area $A_r$ (in the range of $r_r = #qty("1", "cm") ... #qty("10", "cm")$) for different thrust amounts and the resulting effect on the energy required.

#todo[note that larger propellers also mean larger rotational inertia and thus slower control response]

#figure(
  placement: auto,
  {
    mt-energy-3combined()
    caption-rule
  },
  caption: flex-caption(
    [Outflow velocity and orifice area effects on energy consumption],
    [Relation of orifice area $A_r$ and outflow velocity $v_3$ to energy $E_"kin"$ required given a fixed amount of thrust $F$.],
  ),
) <fig:v3_A_energy>

We will come back to this later, as this has an interesting impact on actuator design decisions, specifically in the choice of motor and propeller pairs.

#todo[come back to this later and explain the rotor and motor combinations]


// MAXIMUM PROPELLER EFFICIENCY / BETZ LIMIT

=== Maximum propeller efficiency

Enhancing on #name[Froude]'s and #name[Rankine]'s momentum theory, the maximum efficiency of a propeller is given as the #name[Lanchester]-#name[Betz]-#name[Joukowsky]#footnote[Although french spelling #name[Joukowsky] is used here, note that the transcription #name[Zhukovski] from the russian spelling is also commonly used @WE:WE218.] limit (discovered independently between 1915 and 1920, but generally attributed to #name[Betz] as the #name[Betz] limit @WE:WE218).


// CALCULATION OF DYNAMIC THRUST USING C_t, C_p, K_t, Kp

=== Dynamic thrust

In comparison to @eq:thrust_mdot #vpageref(<eq:thrust_mdot>), an optimized equation for the _dynamic thrust_#index("thrust!dynamic") is given as

$ T_d = c_T 1/2 rho A_b v_b^2 $

where $v_b$ is the lateral blade velocity (e.g. tip velocity) and $A_b$ the blade area @physing[chapter 1.6.3].

#todo[does BEMT get us here?]

// MOMENTUM THEORY / STRAHLTHEORIE

=== Relating momentum theory to motor speed <subsec:momentum_theory_to_motor>

#todo(size: 1.4em, color: red.lighten(50%))[Now how does this relate to rotor speed?]

// OLD NONSENSE

=== Old nonsense

#[
#set text(lang: "de", region: none)

#quote(block: true)[
  Ein Senkrechtstarter kann nur dann senkrecht abheben, wenn die Schubkraft größer ist als die Gewichtskraft des Flugzeugs, siehe auch Schub-Gewicht-Verhältnis. Bei einem 17 Tonnen schweren Hawker Siddeley Harrier z. B. reichen die 200 kN aus seinem Triebwerk aus, um ihn vertikal zu beschleunigen. Bei Starrflügelflugzeugen muss der Schub nur einen Bruchteil des Eigengewichts betragen, da der Flügel den anderen Teil des Eigengewichtes „trägt“.
]
#todo[http://de.wikipedia.org/wiki/Schub]

#todo[two different types of thrust, static and ...]

#text(lang: "en")[
  Static thrust#index("thrust!static")
  is the thrust force applied by an unmoved (_static_) system to its environment.

  In case of a propeller based system, static thrust is the amount of thrust
  produced by the propeller when it is stationary in relation to the ground.

  Viewed differently, it describes the force required to keep a system static if its
  thrusters operate with full power. #todo(inline: false)[what is it good for]
]

#todo[static thrust is also what makes a helicopter hover; the static thrust must at least overpower the gravitational force]

#todo[thrust-to-weight ratio]
#todo[https://quadcopterproject.wordpress.com/static-thrust-calculation/]
#todo[http://electricrcaircraftguy.blogspot.de/2013/09/propeller-static-dynamic-thrust-equation.html]

#v(1em)

Aus der Drehzahl
$N$ #withunit("min^-1")
des Motors, sowie des Radius
$r$ #withunit("m")
der Luftschraube lässt sich die Winkelgeschwindigkeit
$omega_R$ #withunit("rad/s"),
sowie die Blattspitzen- bzw. Umfangsgeschwindigkeit
$U$ #withunit("m/s")
ermitteln:

$
  omega_R &= 2 pi dot N / (60 #unit("s^-1")) approx 0.105 dot N \
  U &= omega_R dot r
$

#todo[Quellen aus #link("http://www.rc-network.de/magazin/artikel_02/art_02-0037/Standschub.pdf")]
#v(1em)

Das die Luftschraube treibende Drehmoment $M$ #withunit("N m") ist mit der Antriebs-/Wellenleistung $P_W$ #withunit("W") des Motors über den Zusammenhang

$ R_W = M dot omega_R $

gekoppelt.

#v(1em) #todo[Zusammenhang...?]

Die vom Motor auf den Propeller übertragene Schubleistung $P_S$ lässt sich über die Formel

$ P_S = K_p dot (N / 1000)^(K_t) $

ermitteln. $K_p$ entspricht hierbei der Propellerkonstante, $K_t$ einer Leistungskonstante #todo(inline: false)[Quelle finden, #link("http://quadcopterproject.wordpress.com/static-thrust-calculation/") ist da sehr vage] und $N$ #withunit("min^-1") der bekannten Drehzahl des Motors.

#v(1em)

Alternativ kann unter Vorgabe des Durchmessers $D$ #text(fill: darkgray, $"in"$) der Luftschraube und ihrer Steigung $S$ #text(fill: darkgray, $"in"$) die Leistung bei Propellerkonstanten $1.1 <= K_p <= 1.3$ abgeschätzt werden#todo(inline: false)[Quelle auf #link("http://corsair.flugmodellbau.de/files/proggies/PROPSC~1.TXT")]:

$ P_S = K_p dot (D / (12 "in"\/"ft"))^4 dot S / (12 "in"\/"ft") dot (N / 1000)^3 $

Für eine $14 times 10$-Luftschraube mit den Werten $K_p = 1.118$ und $K_t = 3.20$ bei $N = #qty("12000", "min^-1")$ #todo(inline: false)[Beispiel für APC Electric $14 times 10$ auf #link("http://aircraft-world.com/prod_datasheets/hp/emeter/hp-propconstants.htm")] ergäbe sich folglich:

$
  P_(K_p, K_t) &= 1.118 dot 12^3.20 \
               &= 3175.6 #unit("W") #<equate:revoke> \
  P_"approx" &= 1.118 dot (14 / 12)^4 (10 / 12) dot (12000 / 1000)^3 \
             &= 2982.5 #unit("W") #<equate:revoke>
$

Für kleinere Werte von $K_p$ weichen die Ergebnisse jedoch stark voneinander ab, so dass das Beispiel $K_p = 0.187$ und $K_t = 3.30$ bei $N = #qty("12000", "min^-1")$ #todo(inline: false)[APC SLOWFLY $10 times 4.7$] nicht funktioniert:

$
  P_(K_p, K_t) &= 0.187 dot 12^3.30 \
               &= 681 #unit("W") #<equate:revoke> \
  P_"approx" &= 0.187 dot (14 / 12)^4 (10 / 12) dot (12000 / 1000)^3 \
             &= 61 #unit("W") #<equate:revoke>
$

#v(1em)

Gemäß @standschub kann der Schub einer Luftschraube wie folgt ermittelt werden:

$
  K_p &= 0.0856 dot H / D - 0.0091 \
  P_S &= K_p dot rho (N / 60)^3 dot D^5 \
  F_S &= 0.67 root(3, rho / 2 dot pi dot D^2 dot P_S^2)
$

wobei $H, D$ #withunit("cm") und $H$ #withunit("m") angenommen werden.

#todo[Großer Blödsinn, Formeln nochmal abtippen.]
]
