# coding: utf-8
import sympy as sp

# Relationship between voltage (V), current (i), frequency (f) and capacitance
# (C).
#
#          -I*i
#     V = --------
#         2*pi*C*f
capacitive_load_func = sp.Eq(sp.Symbol('V'),
                             sp.sympify('i / (I * 2 * pi * f * C)'))
