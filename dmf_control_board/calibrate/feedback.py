# coding: utf-8
r'''
This module contains code for general feedback on each version of the control
board hardware.  The models used here apply to both of the following feedback
circuits on the control board:

 - High-voltage.
 - Impedance.
'''
import pandas as pd
import sympy as sp


def z_transfer_functions():
    r'''
    Return a symbolic equality representation of the transfer function of RMS
    voltage measured by either control board analog feedback circuits.

    According to the figure below, the transfer function describes the
    following relationship:

          # Hardware V1 #                        # Hardware V2 #

            V₂      V₁                               V₂   Z₁
            ── = ───────                             ── = ──
            Z₂   Z₁ + Z₂                             V₁   Z₂

    where $V_{1}$ denotes the high-voltage actuation signal from the amplifier
    output and $V_{2}$ denotes the signal sufficiently attenuated to fall
    within the measurable input range of the analog-to-digital converter
    _(approx. 5V)_.  The feedback circuits for control board hardware version 1
    and hardware version 2 are shown below.

          # Hardware V1 #                        # Hardware V2 #

          V_1 @ frequency                        V_1 @ frequency
              ┯                                      ┯
            ┌─┴─┐                                  ┌─┴─┐    ┌───┐
            │Z_1│                                  │Z_1│  ┌─┤Z_2├─┐
            └─┬─┘                                  └─┬─┘  │ └───┘ │
              ├───⊸ V_2                              │    │  │╲   ├───⊸ V_2
            ┌─┴─┐                                    └────┴──│-╲__│
            │Z_2│                                         ┌──│+╱
            └─┬─┘                                         │  │╱
             ═╧═                                          │
              ¯                                          ═╧═
                                                          ¯

    Notes
    -----

     - The symbolic equality can be solved for any symbol, _e.g.,_ $V_{1}$ or
       $V_{2}$.
     - A symbolically solved representation can be converted to a Python function
       using [`sympy.utilities.lambdify.lambdify`][1], to compute results for
       specific values of the remaining parameters.

    [1]: http://docs.sympy.org/dev/modules/utilities/lambdify.html
    '''
    # Define transfer function as a symbolic equality using SymPy.
    V1, V2, Z1, Z2 = sp.symbols('V1 V2 Z1 Z2')
    xfer_funcs = pd.Series([sp.Eq(V2 / Z2, V1 / (Z1 + Z2)),
                            sp.Eq(V2 / V1, Z2 / Z1)],
                           # Index by hardware version.
                           index=[1, 2])
    xfer_funcs.index.name = 'Hardware version'
    return xfer_funcs


def rc_transfer_function(eq):
    '''
    Substitute resistive and capacitive components for `Z1` and `Z2` in the
    provided equation, where $Z$ is equivalent to parallel resistive and
    capacitive impedances, as shown below.

                                        Z_C
            ┌───┐                    ┌──┤ ├──┐
          ┄─┤ Z ├─┄                 ┄┤  Z_R  ├┄
            └───┘                    └─/\/\/─┘

    See the definitions of $Z_R$ _(resistive impedance)_ and $Z_C$
    _(capacitive impedance)_ [here][1], where $\omega$ is the
    [angular frequency][2].

    Specifically,

                1
        Z = ─────────
                    1
            ⅈ⋅C⋅ω + ─
                    R

    [1]: http://en.wikipedia.org/wiki/Electrical_impedance#Device_examples
    [2]: http://en.wikipedia.org/wiki/Angular_frequency
    '''
    R1, C1, R2, C2, omega = sp.symbols('R1 C1 R2 C2 omega')
    return eq.subs([('Z2', 1 / (1 / R2 + sp.I * omega * C2)),
                    ('Z1', 1 / (1 / R1 + sp.I * omega * C1))])


def control_board_transfer_functions(xfer_funcs, solve_for, symbolic=False):
    r'''
    Return a numeric function to solve for one of `('V1', 'V2')`, of the
    form:

        f(V, R1, C1, R2, C2, frequency)

    where `V` corresponds to the known variable out of `('V1', 'V2')`.
    '''
    args = ['V1', 'V2', 'R1', 'C1', 'R2', 'C2', 'f']
    if solve_for not in args:
        raise ValueError('''`solve_for` must be one of `('V1', 'V2')`.''')
    else:
        args.remove(solve_for)

    f = sp.symbols('f')
    sym_funcs = xfer_funcs.map(lambda x: sp.Abs(sp.solve(x, solve_for)[0]
                                                .subs('omega', 2 * sp.pi * f)))
    if symbolic:
        # Return symbolic functions corresponding to the specified variable.
        return sym_funcs
    else:
        # Return numeric function instantiations to compute the specified
        # variable.  We specify that `numpy` functions should be used, when
        # necessary.
        return sym_funcs.map(lambda F:
                             sp.utilities.lambdify(', '.join(args), F,
                                                   'numpy'))
