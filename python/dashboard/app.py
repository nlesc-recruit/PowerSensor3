import os
import json

from dash import Dash, html, dcc
from dash.dependencies import Input, Output, State
import plotly.graph_objects as go
import numpy as np

import powersensor


app = Dash('PowerSensor3')

ps = None

INTERVAL = 100  # ms
MAX_TIME = 10  # s


app.layout = html.Div([
    html.H1('PowerSensor 3',
            style={'textAlign': 'center'}),

    html.Div([html.Div('Select device and press "Connect" to get started'),
              # dcc.Input(id='set-device', type='text', value='/dev/ttyACM0'),
              dcc.Input(id='set-device', type='text', value='/dev/cu.usbmodem386A367F32371'),
              html.Button(id='connect', children='Connect'),
              html.Div(id='connection-status'),
              dcc.Store(id='t0', data='0')]),

    html.Div([html.Button('Toggle plotting', id='run-plotter-button'),
              dcc.Store(id='run-plotter', data=json.dumps(False))]),

    html.Div([dcc.Graph(id='graph', figure=go.Figure()),
              dcc.Interval(id='interval', interval=INTERVAL)])
])


@app.callback(Output('connection-status', 'children'),
              Output('t0', 'data'),
              Output('connect', 'children'),
              Input('connect', 'n_clicks'),
              State('set-device', 'value'),
              State('t0', 'data'))
def connect(n_clicks, device, t0):
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


@app.callback(Output('run-plotter', 'data'),
              Input('run-plotter-button', 'n_clicks'),
              State('run-plotter', 'data'))
def toggle_plotting(n_clicks, data):
    if n_clicks is None:
        return data
    run_plotter = json.loads(data)
    run_plotter = not run_plotter
    return json.dumps(run_plotter)


@app.callback(Output('graph', 'figure'),
              Input('interval', 'n_intervals'),
              State('graph', 'figure'),
              State('run-plotter', 'data'),
              State('t0', 'data'))
def update_plot(n_intervals, fig, run_plotter, t0):
    run_plotter = json.loads(run_plotter)
    t0 = json.loads(t0)
    # no updates if not connected or plotting disabled
    if (ps is None) or (not run_plotter) or (n_intervals is None):
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
