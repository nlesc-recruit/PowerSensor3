import json

from dash import html, dcc
import plotly.graph_objects as go

from callbacks import app

INTERVAL = 100  # ms


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


if __name__ == '__main__':
    app.run_server(debug=True)
