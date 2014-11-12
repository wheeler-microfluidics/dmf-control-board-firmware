# coding: utf-8
import pandas as pd
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
import scipy.optimize as optimize

from microdrop_utility import Version, is_float


def rc_transfer_function(eq):
    '''
    Substitute resistive and capacitive components for `Z1` and `Z2`
    in the provided equation.

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


def z_transfer_functions():
    r'''
    Return a symbolic equality representation of the transfer function of RMS
    voltage measured by control board high-voltage feedback analog input
    relative to the actual high-voltage RMS value.

    According to the figure below, the transfer function describes the
    following relationship:

          # Hardware V1 #                        # Hardware V2 #

            V₂      V₁                               V₂   Z₁
            ── = ───────                             ── = ──
            Z₂   Z₁ + Z₂                             V₁   Z₂

    where $V_{1}$ denotes the high-voltage signal from the amplifier output
    and $V_{2}$ denotes the signal sufficiently attenuated to fall within the
    measurable input range of the analog-to-digital converter _(approx. 5V)_.

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

    where $V_{1}$ denotes the high-voltage signal from the amplifier output
    and $V_{2}$ denotes the signal sufficiently attenuated to fall within the
    measurable input range of the analog-to-digital converter _(approx. 5V)_.

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


def control_board_hv_transfer_functions():
    r'''
    Return a `pandas.Series`, indexed by control board hardware version,
    containing the symbolic transfer function corresponding to the respective
    _high-voltage_ feedback measurement circuit layout.

    In the high-voltage feedback measurement circuit, the $Z_1$ impedance is
    assumed to be purely resistive _(i.e., a capacitive load of zero)_.
    '''
    xfer_funcs = z_transfer_functions()
    rc_xfer_funcs = xfer_funcs.map(rc_transfer_function)
    return rc_xfer_funcs.map(lambda x: x.subs('C1', 0))


def control_board_feedback_transfer_functions():
    r'''
    Return a `pandas.Series`, indexed by control board hardware version,
    containing the symbolic transfer function corresponding to the respective
    _capacitive load_ feedback measurement circuit layout.

    In the capacitive load feedback measurement circuit, the $Z_1$ impedance is
    assumed to be purely capacitive _(i.e., zero resistive load)_.
    '''
    xfer_funcs = z_transfer_functions()
    rc_xfer_funcs = xfer_funcs.map(rc_transfer_function)
    return rc_xfer_funcs.map(lambda x: x.subs('R1', 0))


def control_board_transfer_functions(xfer_funcs, solve_for, symbolic=False):
    r'''
    Return a numeric function to solve for one of `('V1', 'V2')`, of the
    form:

        f(V, R1, frequency, R, C)

    where `V` corresponds to the known variable out of `('V1', 'V2')`.
    '''
    if solve_for == 'V1':
        V = 'V2'
    elif solve_for == 'V2':
        V = 'V1'
    else:
        raise ValueError('''`solve_for` must be one of `('V1', 'V2')`.''')

    f = sp.symbols('f')
    sym_funcs = xfer_funcs.map(lambda x: sp.Abs(sp.solve(x, solve_for)[0]
                                                .subs('omega', 2 * sp.pi * f)))
    if symbolic:
        # Return symbolic functions corresponding to the specified variable.
        return sym_funcs
    else:
        # Return numeric function instantiations to compute the specified
        # variable.
        return sym_funcs.map(lambda F:
                             sp.utilities.lambdify(V + ', R1, C1, R2, C2, f',
                                                   F, 'numpy'))


def measure_board_rms(control_board, n_samples=10, sampling_ms=10,
                      delay_between_samples_ms=0):
    '''
    Read RMS voltage samples from control board high-voltage feedback circuit.
    '''
    results = control_board.measure_impedance(10, 10, 0, True, True, [])
    data = pd.DataFrame({'board measured V': results.V_hv})
    data['divider resistor index'] = results.hv_resistor
    return data


def find_good(control_board, actuation_steps, resistor_index, start_index,
              end_index):
    '''
    Use a binary search over the range of provided actuation_steps to find the
    maximum actuation voltage that is measured by the board feedback circuit
    using the specified feedback resistor.
    '''
    lower = start_index
    upper = end_index
    while lower < upper - 1:
        index = lower + (upper - lower) / 2
        v = actuation_steps[index]
        control_board.set_waveform_voltage(v)
        data = measure_board_rms(control_board)
        valid_data = data[data['divider resistor index'] >= 0]

        if (valid_data['divider resistor index'] <
                resistor_index).sum():
            # We have some measurements from another resistor.
            upper = index
        else:
            lower = index
    control_board.set_waveform_voltage(actuation_steps[lower])
    data = measure_board_rms(control_board)
    return lower, data


def resistor_max_actuation_readings(control_board, frequencies,
                                    oscope_reading_func):
    '''
    For each resistor in the high-voltage feedback resistor bank, read the
    board measured voltage and the oscilloscope measured voltage for an
    actuation voltage that nearly saturates the feedback resistor.

    By searching for an actuation voltage near saturation, the signal-to-noise
    ratio is minimized.
    '''
    # Set board amplifier gain to 1.
    # __NB__ This is likely _far_ lower than the actual gain _(which may be a
    # factor of several hundred)_..
    control_board.set_waveform_voltage(0)
    control_board.auto_adjust_amplifier_gain = False
    control_board.amplifier_gain = 1.

    # Set waveform voltage to a low value and obtain the corresponding
    # oscilloscope reading to calculate an approximate gain of the amplifier.
    target_voltage = 0.1
    control_board.set_waveform_voltage(target_voltage)
    oscope_rms = oscope_reading_func()
    estimated_amplifier_gain = oscope_rms / target_voltage

    # Based on the maximum amplified RMS voltage, define a set of actuation
    # voltages to search when performing calibration.
    max_post_gain_V = 150.  # TODO: Read this value from device.
    max_actuation_V = max_post_gain_V / estimated_amplifier_gain
    actuation_steps = np.linspace(0.005, max_actuation_V, num=100)

    # Define frequency/resistor index pairs to take measurements at.
    conditions = pd.DataFrame([[r, f] for r in range(2, -1, -1)
                               for f in frequencies],
                              columns=['resistor index', 'frequency'])

    # Define function to process each frequency/resistor index pair.
    def max_actuation_reading(x):
        '''
        Measure maximum board RMS voltage using specified feedback resistor, at
        the specified frequency.

        Request corresponding oscilloscope RMS voltage reading.
        '''
        r = x['resistor index'].values[0]
        f = x['frequency'].values[0]
        control_board.set_waveform_frequency(f)

        actuation_index, data = find_good(control_board, actuation_steps, r, 0,
                                          len(actuation_steps) - 1)
        board_measured_rms = data.loc[data['divider resistor index']
                                    >= 0, 'board measured V'].mean()
        oscope_rms = oscope_reading_func()
        return pd.DataFrame([[r, f, actuation_index, board_measured_rms,
                            oscope_rms]],
                            columns=['resistor index', 'frequency',
                                    'actuation index', 'board measured V',
                                    'oscope measured V'])

    # Return board-measured RMS voltage and oscilloscope-measured RMS voltage
    # for each frequency/feedback resistor pair.
    return (conditions.groupby(['resistor index', 'frequency'])
            .apply(max_actuation_reading).reset_index(drop=True))


def fit_feedback_params(control_board, max_resistor_readings):
    '''
    Fit model of control board high-voltage feedback resistor and
    parasitic capacitance values based on measured voltage readings.
    '''
    hardware_version = Version.fromstring(control_board
                                          .hardware_version())
    R1 = 10e6

    # Get transfer function to compute the amplitude of the high-voltage input
    # to the control board _(i.e., the output of the amplifier)_ based on the
    # attenuated voltage measured by the analog-to-digital converter on the
    # control board.
    #
    # The signature of the transfer function is:
    #
    #     H(V1, R1, C1, R2, C2, f)
    #
    # See the `z_transfer_functions` function docstring for definitions of the
    # parameters based on the control board major version.
    f = get_transfer_function(hardware_version.major, 'V2')
    e = lambda p, freq, y, R1: f(1, R1, 0, p[0], p[1], freq) - y

    def fit_resistor_params(x):
        resistor_index = x['resistor index'].values[0]
        x['attenuation'] = x['board measured V'] / x['oscope measured V']
        p0 = [control_board.calibration.C_hv[resistor_index],
              control_board.calibration.R_hv[resistor_index]]

        p1, success = optimize.leastsq(e, p0,
                                       args=(x['frequency'],
                                             x['attenuation'], R1))
        return pd.DataFrame([p0 + p1.tolist()],
                            columns=['original C', 'original R',
                                     'fitted C', 'fitted R']).T

    results = (max_resistor_readings
               [max_resistor_readings['resistor index'] >= 0]
               .groupby(['resistor index']).apply(fit_resistor_params))
    data = results.unstack()
    data.columns = data.columns.droplevel()
    return data


def plot_feedback_params(transfer_func, max_resistor_readings,
                         feedback_params, axis=None):
    '''
    Plot the effective attenuation _(i.e., gain less than 1)_ of the control
    board measurements of high-voltage AC input according to:

     - AC signal frequency.
     - feedback resistor used _(varies based on amplitude of AC signal)_.

    Each high-voltage feedback resistor (unintentionally) forms a low-pass
    filter, resulting in attenuation of the voltage measured on the control
    board.  The plot generated by this function plots each of the following
    trends for each feedback resistor:

     - Oscilloscope measurements.
     - Previous model of attenuation.
     - Newly fitted model of attenuation, based on oscilloscope readings.
    '''
    R1 = 10e6

    # Since the feedback circuit changed in version 2 of the control board, we
    # use the transfer function that corresponds to the current control board
    # version that the fitted attenuation model is based on.
    if axis is None:
        fig = plt.figure()
        axis = fig.add_subplot(111)
    colors = axis._get_lines.color_cycle
    markers = MarkerStyle.filled_markers

    def plot_resistor_params(args):
        resistor_index, x = args
        color = colors.next()

        F = feedback_params.loc[resistor_index]
        axis.loglog(x['frequency'],
                    transfer_func(1, R1, 0,
                                  F['original R'], F['original C'],
                                  x['frequency']), color=color,
                    linestyle='--', label='R$_{%d}$ (previous fit)' %
                    resistor_index)

        axis.loglog(x['frequency'],
                    transfer_func(1, R1, 0,
                                  F['fitted R'], F['fitted C'],
                                  x['frequency']), color=color,
                    linestyle='-', label='R$_{%d}$ (new fit)' % resistor_index,
                    alpha=0.6)
        attenuation = x['board measured V'] / x['oscope measured V']
        axis.plot(x['frequency'], attenuation, color='none',
                  marker=markers[resistor_index % len(markers)],
                  label='R$_{%d}$ (scope measurements)' % resistor_index,
                  linestyle='none', markeredgecolor=color, markeredgewidth=2,
                  markersize=8)
        return 0

    map(plot_resistor_params, max_resistor_readings.groupby('resistor index'))
    legend = axis.legend(ncol=3)
    legend.draw_frame(False)
    axis.set_xlabel('Frequency (Hz)')
    axis.set_ylabel(r'$\frac{V_{BOARD}}'
                    r'{V_{SCOPE}}$', fontsize=25)


def get_transfer_function(hardware_major_version, solve_for, symbolic=False):
    '''
    Return a numeric function to solve for one of `('V_1', 'V_2')`, of the
    form:

        f(V, R1, R2, C1, C2, frequency)

    where `V` corresponds to the known variable out of `('V_1', 'V_2')`.
    '''
    hv_xfer_funcs = control_board_hv_transfer_functions()
    xfer_funcs = control_board_transfer_functions(hv_xfer_funcs, solve_for,
                                                  symbolic=symbolic)
    return xfer_funcs[hardware_major_version]


def update_control_board_calibration(control_board, fitted_params):
    '''
    Update the control board with the specified fitted parameters.
    '''
    def update_control_board_params(x):
        resistor_index = int(x.name)
        control_board.set_series_resistor_index(0, resistor_index)
        control_board.set_series_resistance(0, abs(x['fitted R']))
        control_board.set_series_capacitance(0, abs(x['fitted C']))

    # For each resistor index, update the control board with the new
    # fitted capacitor and resistor values.
    fitted_params[['fitted C', 'fitted R']].apply(
        update_control_board_params, axis=1)

    # Reconnnect to update calibration data.
    control_board.connect()
