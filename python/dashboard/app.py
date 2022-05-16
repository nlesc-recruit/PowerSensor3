import os
from time import sleep

from dash import Dash, html, dcc
from dash.dependencies import Input, Output, State
import plotly.subplots
import plotly.graph_objects as go
import plotly.express as px
import numpy as np
import pandas as pd

import powersensor


app = Dash('PowerSensor3')

# TODO: should use dcc.Store instead of globals wherever possible
ps = None
run_plotter = False
t0 = 0

INTERVAL = 100  # ms
MAX_TIME = 10  # s


app.layout = html.Div([
    html.H1('PowerSensor 3',
            style={'textAlign': 'center'}),

    html.Div([html.Div('Select device and press "Connect" to get started'),
              # dcc.Input(id='set-device', type='text', value='/dev/ttyACM0'),
              dcc.Input(id='set-device', type='text', value='/dev/cu.usbmodem386A367F32371'),
              html.Button('Connect', id='connect'),
              html.Div(id='connection-status')]),

    html.Div([html.Button('Toggle plotting', id='run-plotter'),
              html.Div(id='plotting-status')]),

    html.Div([dcc.Graph(id='graph', figure=go.Figure()),
              dcc.Interval(id='interval', interval=INTERVAL, n_intervals=0)])
])


@app.callback(Output('connection-status', 'children'),
              Input('connect', 'n_clicks'),
              State('set-device', 'value'))
def connect(n_clicks, device):
    global ps, t0
    if n_clicks is None:
        return "Status: Not connected"
    if not os.path.exists(device):
        return f"Status: No such device: {device}"
    # connect to powersensor
    ps = powersensor.PowerSensor(device)
    t0 = ps.read().time_at_read
    return "Status: Connected"


@app.callback(Output('plotting-status', 'children'),
              Input('run-plotter', 'n_clicks'))
def toggle_plotting(n_clicks):
    global run_plotter
    if n_clicks is not None:
        run_plotter = not run_plotter
    if run_plotter:
        return "Plotting enabled"
    else:
        return "Plotting disabled"


@app.callback(Output('graph', 'figure'),
              Input('interval', 'n_intervals'),
              State('graph', 'figure'))
def update_plot(n_intervals, fig):
    # no updates if not connected or plotting disabled
    if (ps is None) or (not run_plotter):
        return fig

    fig = go.Figure(fig)
    fig.update_layout(xaxis_title='Time (s)', yaxis_title='Total power (W)')

    # read powersensor
    state = ps.read()
    time = state.time_at_read - t0
    power = sum(np.array(state.current) * np.array(state.voltage))

    if not fig.data:
        # nothing in the figure, add a new trace
        fig.add_trace({'x': [time], 'y': [power]})
    else:
        # add new point to data
        x = fig.data[0].x + (time, )
        y = fig.data[0].y + (power, )
        # only keep last XX seconds
        min_idx = np.searchsorted(x, time - MAX_TIME)
        fig.data[0].x = x[min_idx:]
        fig.data[0].y = y[min_idx:]
    return fig


if __name__ == '__main__':
    app.run_server(debug=True)
