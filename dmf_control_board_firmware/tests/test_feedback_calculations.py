import numpy as np
import pandas as pd
from path_helpers import path

def compare_results_to_reference(method, reference_results_file, id=1, filter_order=None):
    input_file = path(__file__).parent / path('FeedbackResults') / \
        path('input_1.pickle')

    data = input_file.pickle_load()

    # add absolute path
    reference_results_file = path(__file__).parent / path('FeedbackResults') / \
        reference_results_file

    f = pd.HDFStore(reference_results_file, 'r')
    reference_results_df = f['/root']
    f.close()

    order = filter_order
    if filter_order is None:
        # filter_order=None is represented as -1
        order = -1

    # filter the results based on id and filter_order
    reference_results_df = reference_results_df[(reference_results_df['id'] == id) &
                                                (reference_results_df['filter_order'] == order)]

    if method in ['force', 'V_actuation']:
        results = eval('data.%s()' % method)
    elif method in ['dxdt']:
        t, results = eval('data.%s(filter_order=%s)' % (method, filter_order))
    else:
        results = eval('data.%s(filter_order=%s)' % (method, filter_order))

    # re-cast masked array as normal array
    results = np.array(results)

    max_diff = np.max(np.abs(reference_results_df[method].values - results))
    if max_diff > 1e-14:
        print max_diff
        assert max_diff == 0

    nan_dif = np.isnan(reference_results_df[method].values) ^ np.isnan(results)
    if np.any(nan_dif):
        print "Differences exist in the 'np.isnan(x)' status for some values."
        assert False

def test_V_actuation():
    compare_results_to_reference('V_actuation', 'reference_results.hdf')

def test_x_position():
    compare_results_to_reference('x_position', 'reference_results.hdf')

def test_force():
    compare_results_to_reference('force', 'reference_results.hdf')

def test_Z_device():
    compare_results_to_reference('Z_device', 'reference_results.hdf')

def test_Z_device_filter_order_3():
    compare_results_to_reference('Z_device', 'reference_results.hdf', filter_order=3)

def test_capacitance():
    compare_results_to_reference('capacitance', 'reference_results.hdf')

def test_velocity():
    compare_results_to_reference('dxdt', 'reference_results_velocity.hdf')

def test_velocity_filter_order_3():
    compare_results_to_reference('dxdt', 'reference_results_velocity.hdf', filter_order=3)

def generate_feedback_results_reference(data, id):
    input_file = path(__file__).parent / path('FeedbackResults') / \
        path('input_%s.pickle' % id)

    # save a pickled version of the feedback results object
    input_file.pickle_dump(data, -1)

    for fname in ['reference_results.hdf', 'reference_results_velocity.hdf']:
        reference_results_file = path(__file__).parent / \
            path('FeedbackResults') / path(fname)

        if reference_results_file.exists():
            f = pd.HDFStore(reference_results_file, 'r')
            reference_results_df = f['/root']
            f.close()

            # remove any existing data with the same id
            reference_results_df = reference_results_df[reference_results_df['id'] != id]
        else:
            reference_results_df = pd.DataFrame()

        for filter_order in [None, 3]:
            if fname == 'reference_results_velocity.hdf':
                t, dxdt = data.dxdt(filter_order=filter_order)
                d = {'dxdt': dxdt,
                     'time': t,
                }
            else:
                d = {'V_actuation': data.V_actuation(),
                     'force': data.force(),
                     'Z_device': data.Z_device(filter_order=filter_order),
                     'capacitance': data.capacitance(filter_order=filter_order),
                     'x_position': data.x_position(filter_order=filter_order),
                     'time': data.time,
                }

            results_df = pd.DataFrame(d)
            results_df['id'] = id

            # store filter_order=None as -1
            if filter_order is None:
                results_df['filter_order'] = -1
            else:
                results_df['filter_order'] = filter_order

            reference_results_df = reference_results_df.append(results_df)

        # save an hdf file containing the reference feedback results calculations
        reference_results_df.to_hdf(reference_results_file, '/root', complib='blosc', complevel=2)

def test_get_window_size():
    input_file = path(__file__).parent / path('FeedbackResults') / \
        path('input_1.pickle')
    data = input_file.pickle_load()
    assert data._get_window_size() == 21.0

