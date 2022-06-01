import os
import json

import dash
import numpy as np
from plotly.subplots import make_subplots
import plotly.graph_objects as go

import powersensor

app = dash.Dash('PowerSensor3')
ps = None
MAX_TIME = 10  # s


@app.callback(dash.Output('connection-status', 'children'),
              dash.Output('t0', 'data'),
              dash.Output('connect', 'children'),
              dash.Input('connect', 'n_clicks'),
              dash.State('set-device', 'value'),
              dash.State('t0', 'data'))
def toggle_connect(n_clicks, device, t0):
    global ps
    status = 'Status: Not connected'
    button_text = 'Connect'

    if n_clicks is None:
        ps = None
    elif ps is None:
        # connect to powersensor
        if not os.path.exists(device):
            status = f'Status: No such device: {device}'
        else:
            ps = powersensor.PowerSensor(device)
            status = 'Status: Connected'
            if json.loads(t0) == 0:
                t0 = json.dumps(ps.read().time_at_read)
            button_text = 'Disconnect'
    else:
        # disconnect
        ps = None
    return status, t0, button_text


@app.callback(dash.Output('run-plotter', 'data'),
              dash.Input('run-plotter-button', 'n_clicks'),
              dash.State('run-plotter', 'data'))
def toggle_plotting(n_clicks, data):
    if n_clicks is None:
        return data
    run_plotter = json.loads(data)
    run_plotter = not run_plotter
    return json.dumps(run_plotter)


@app.callback(dash.Output('fig-power', 'figure'),
              dash.Output('fig-current', 'figure'),
              dash.Output('fig-voltage', 'figure'),
              dash.Input('interval', 'n_intervals'),
              dash.State('fig-power', 'figure'),
              dash.State('fig-current', 'figure'),
              dash.State('fig-voltage', 'figure'),
              dash.State('run-plotter', 'data'),
              dash.State('t0', 'data'),
              )
def update_plot(n_intervals, fig_power, fig_current, fig_voltage, run_plotter, t0):
    run_plotter = json.loads(run_plotter)
    t0 = json.loads(t0)

    # no updates if not connected or plotting disabled
    if (n_intervals is None) or (ps is None) or (not run_plotter):
        return fig_power, fig_current, fig_voltage

    fig_power = go.Figure(fig_power)
    fig_current = go.Figure(fig_current)
    fig_voltage = go.Figure(fig_voltage)
    figures = [fig_power, fig_current, fig_voltage]

    yaxis_labels = ('Total power (W)', 'Current (A)', 'Voltage (V)')
    for idx, fig in enumerate(figures):
        fig.update_layout(xaxis_title='Time (s)', yaxis_title=yaxis_labels[idx])

    # read powersensor
    state = ps.read()
    time = state.time_at_read - t0
    current = np.array(state.current)
    voltage = np.array(state.voltage)
    power = sum(current * voltage)
    inactive_sensors = np.isclose(voltage, 0)

    if not fig_power.data:
        # nothing in the figures, add a new traces
        fig_power.add_trace({'x': [time], 'y': [power]})
        for idx, a in enumerate(current):
            if inactive_sensors[idx]:
                continue
            fig_current.add_trace(go.Scatter(x=[time],
                                             y=[a],
                                             name=str(idx)))
        for idx, v in enumerate(voltage):
            if inactive_sensors[idx]:
                continue
            fig_voltage.add_trace(go.Scatter(x=[time],
                                             y=[v],
                                             name=str(idx)))

    else:
        # add new points to figures
        values = [[power], current, voltage]
        for fig_idx, fig in enumerate(figures):
            for trace_idx in range(len(fig.data)):
                if inactive_sensors[idx] and (fig_idx != 0):
                    continue
                x = fig.data[trace_idx].x + (time, )
                y = fig.data[trace_idx].y + (values[fig_idx][trace_idx], )
                # only keep last MAX_TIME seconds
                min_idx = np.searchsorted(x, time - MAX_TIME)
                fig.data[trace_idx].x = x[min_idx:]
                fig.data[trace_idx].y = y[min_idx:]

    return figures
