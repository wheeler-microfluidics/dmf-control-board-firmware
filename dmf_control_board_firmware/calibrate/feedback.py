# coding: utf-8
r'''
This module contains code for general feedback on each version of the control
board hardware.  The models used here apply to both of the following feedback
circuits on the control board:

 - High-voltage.
 - Impedance.
'''
import re
from collections import OrderedDict, Iterable
from functools32 import lru_cache

import pandas as pd
import sympy as sp


def limit_default(equation, symbol_names, default, **kwargs):
    return swap_default('limit_default', equation, symbol_names, default,
                        **kwargs)


def subs_default(equation, symbol_names, default, **kwargs):
    return swap_default('subs', equation, symbol_names, default, **kwargs)


@lru_cache(maxsize=500)
def _subs(eq, symbol, value):
    return eq.subs(symbol, value)


@lru_cache(maxsize=500)
def _limit(eq, *args, **kwargs):
    if isinstance(eq, sp.Eq):
        # The provided equation is an equality, so we need to process the limit
        # of each side independently.
        return sp.Eq(sp.limit(eq.lhs, *args, **kwargs),
                     sp.limit(eq.rhs, *args, **kwargs))
    else:
        return sp.limit(*args, **kwargs)


def swap_default(mode, equation, symbol_names, default, **kwargs):
    '''
    Given a `sympy` equation or equality, along with a list of symbol names,
    substitute the specified default value for each symbol for which a value is
    not provided through a keyword argument.

    For example, consider the following equality:

    >>> sp.pprint(H)
    V₂   Z₂
    ── = ──
    V₁   Z₁

    Let us substitute a default value of 1 for terms Z1 and Z2:

    >>> sp.pprint(subs_default(H, ['Z1', 'Z2'], 1))
    V₂
    ── = 1
    V₁

    Now, let us specify a default value of 1 for terms Z1 and Z2, but provide
    an overriding value for Z1:

    >>> sp.pprint(subs_default(H, ['Z1', 'Z2'], 1, Z1=4))
    V₂
    ── = 1/4
    V₁

    Note that keyword arguments for terms not specified in the list of symbol
    names are ignored:

    >>> sp.pprint(subs_default(H, ['Z1', 'Z2'], 1, Z1=4, Q=7))
    V₂
    ── = 1/4
    V₁
    '''
    if mode == 'subs':
        swap_f = _subs
        default_swap_f = _subs
    elif mode == 'limit':
        swap_f = _limit
        default_swap_f = _subs
    elif mode == 'limit_default':
        swap_f = _subs
        default_swap_f = _limit
    else:
        raise ValueError('''Unsupported mode.  `mode` must be one of: '''
                         '''('subs', 'limit').''')

    result = equation
    for s in symbol_names:
        if s in kwargs:
            if isinstance(kwargs[s], Iterable):
                continue
            else:
                result = swap_f(result, s, kwargs[s])
        else:
            result = default_swap_f(result, s, default)
    return result


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


def rc_transfer_function(eq, Zs=None):
    '''
    Substitute resistive and capacitive components for all `Zs` terms _(all
    terms starting with `Z` by default)_ in the provided equation, where $Z$ is
    equivalent to parallel resistive and capacitive impedances, as shown below.

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
    if Zs is None:
        Zs = [s.name for s in eq.atoms(sp.Symbol) if s.name.startswith('Z')]
    result = eq
    cre_Z = re.compile(r'Z(?P<suffix>.*)')
    sub_params = [(z, sp.symbols('R%s C%s omega' % (s, s)))
                  for z, s in [(Z, cre_Z.match(Z).group('suffix'))
                               for Z in Zs]]
    for Z, (R, C, omega) in sub_params:
        result = result.subs(Z, 1 / (1 / R + sp.I * omega * C))
    return result


@lru_cache(maxsize=500)
def get_transfer_function(hardware_major_version, solve_for=None, Zs=None):
    '''
    Return feedback measurement circuit symbolic transfer function for
    specified control board hardware version as `sympy` equality.

    Keyword arguments:
     - `solve_for`: Rearrange equality with `solve_for` symbol as LHS.
     - `Zs`:
      * List of impedance _(i.e., `Z`)_ to substitute with resistive and
        capacitive terms.
      * By default, all `Z` terms are substituted with corresponding `R` and
        `C` values.
    '''
    xfer_func = z_transfer_functions()[hardware_major_version]
    symbols = OrderedDict([(s.name, s)
                           for s in xfer_func.atoms(sp.Symbol)])
    if Zs is None:
        Zs = [s for s in symbols if s != solve_for and s.startswith('Z')]
    H = rc_transfer_function(xfer_func, Zs)
    if solve_for is None:
        return H
    symbols = OrderedDict([(s.name, s)
                           for s in H.atoms(sp.Symbol)])
    solve_for_symbol = symbols[solve_for]
    solved = sp.Eq(solve_for_symbol, sp.solve(H, solve_for_symbol)[0])
    return solved


def compute_from_transfer_function(hardware_major_version, solve_for,
                                   **kwargs):
    '''
    Positional arguments:

     - `hardware_major_version`:
      * Major version of control board hardware _(1 or 2)_.
     - `solve_for`: Variable in feedback transfer function to solve for.
      * See
      * _e.g.,_ `Z1`, `V1`, `V2`, etc.

    Keyword arguments:

     - `symbolic`: Return `sympy` symbolic equality.
     - Other:
      * Scalar or array-like value to substitute for term with corresponding
        name in transfer function.
      * In the case of the frequency term, `f`, if set to `True`, substitute
        `2 * pi * f` for angular frequency _(i.e., `omega`)_ in transfer function.

    Conventions:

     - Symbols starting with `Z` are assumed to be impedance terms.
     - Symbols starting with `R` are assumed to be resistive impedance terms.
     - Symbols starting with `C` are assumed to be capacitive impedance terms.
     - `omega` is assumed to be an angular frequency term.
     - `f` is assumed to be a frequency term _(i.e., `omega = 2 * pi * f`).

    Either `f` or `omega` may be specified, _not_ both.
    '''
    symbolic = kwargs.pop('symbolic', False)
    # Get list of all `Z` terms provided as keyword arguments.
    Zs = tuple([s for s in kwargs if s != solve_for and s.startswith('Z')])
    # If no `Z` terms were provided, substitute `R` and `C` terms for all `Z`
    # terms in transfer function.
    if not Zs:
        Zs = None
    # Get symbolic feedback measurement transfer function solved for specified
    # term.
    result = get_transfer_function(hardware_major_version, solve_for=solve_for,
                                   Zs=Zs)
    # Get list of resistive terms in transfer function.
    Rs = set([s.name for s in result.atoms(sp.Symbol)
              if s.name.startswith('R')])
    # Get list of capacitive terms in transfer function.
    Cs = set([s.name for s in result.atoms(sp.Symbol)
              if s.name.startswith('C')])
    # Substitute infinite resistance _(i.e., open circuit)_ for any resistive
    # term in the transfer function where a resistance value was not specified
    # as a keyword argument.
    result = limit_default(result, Rs, sp.oo, **kwargs)
    # Substitute zero capacitance _(i.e., open circuit)_ for any capacitive
    # term in the transfer function where a resistance value was not specified
    # as a keyword argument.
    result = limit_default(result, Cs, 0, **kwargs)
    # Substitute frequency term for angular frequency as necessary.
    if 'f' in kwargs:
        result = result.subs('omega', sp.sympify('2 * pi * f'))
        if not isinstance(kwargs['f'], Iterable) and kwargs['f'] == True:
            # Interpret as request to symbolically substitute `2 * pi * f` for
            # `omega`.  Thus, remove `f` before performing value substitution.
            del kwargs['f']
    # Substitute scalar values from keyword arguments into symbolic equation.
    for k, v in kwargs.iteritems():
        if isinstance(v, Iterable):
            continue
        else:
            result = result.subs(k, v)
    if symbolic:
        return result
    else:
        # Create list of unresolved symbols on the right-hand-side of the
        # equality.
        H = result.rhs
        symbols = [s.name for s in H.atoms(sp.Symbol)]
        # Construct numeric equation with the unresolved RHS terms as
        # arguments.
        func = sp.lambdify(', '.join(symbols), sp.Abs(H), 'numpy')
        # Return resulting numeric function evaluated with provided keyword
        # value for each term.
        return func(*[kwargs[s] for s in symbols])
