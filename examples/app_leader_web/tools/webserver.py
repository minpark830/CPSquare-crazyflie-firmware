''' 
    Copyright (C) 2025 CPSquare Lab
    Written by Minwoo Park 
'''

from flask import Flask, request, jsonify, render_template_string
import plotly.graph_objs as go
import plotly
import json

app = Flask(__name__)

leader_position = []
follower_1_position = []
follower_2_position = []
follower_3_position = []

@app.route('/')
def show_position_plot():

    def make_trace(positions, name, color):
        x_pos = [p[0] for p in positions]
        y_pos = [p[1] for p in positions]
        z_pos = [p[2] for p in positions]
        return go.Scatter3d(
            x=x_pos, y=y_pos, z=z_pos,
            mode='markers+lines',
            name=name,
            showlegend=True,
            marker=dict(size=6, color=color, opacity=0.8),
            line=dict(width=2, color=color)
        )

    # Create traces for each agent
    traces = [
        make_trace(leader_position, "Leader", "blue"),
        make_trace(follower_1_position, "Follower 1", "red"),
        make_trace(follower_2_position, "Follower 2", "green"),
        make_trace(follower_3_position, "Follower 3", "orange"),
    ]

    layout = go.Layout(
        title="3D Position Plot: Leader and Followers",
        margin=dict(l=0, r=0, b=0, t=40),
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        ),
        legend=dict(
            bgcolor='rgba(255,255,255,0.7)',
            bordercolor='black',
            borderwidth=1,
            font=dict(size=12)
        )
    )

    fig = go.Figure(data=traces, layout=layout)
    plot_json = json.dumps(fig, cls=plotly.utils.PlotlyJSONEncoder)

    html_template = '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>3D Leader-Follower Plot</title>
        <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    </head>
    <body>
        <h2>Interactive 3D Plot: Leader & Followers</h2>
        <div id="plot" style="width: 100%; height: 90vh;"></div>
        <script>
            var figure = {{ plot_json | safe }};
            Plotly.newPlot('plot', figure.data, figure.layout);
        </script>
    </body>
    </html>
    '''

    return render_template_string(html_template, plot_json=plot_json)


@app.route('/leader/position', methods=['GET', 'POST'])
def handle_leader_position():
    global leader_position
    if request.method == 'POST':
        data = request.get_json()
        if not data or not all(k in data for k in ('x', 'y', 'z')):
            return jsonify({'error': 'Invalid data'}), 400
        
        # Append new position to the list
        leader_position.append([data.get('x'), data.get('y'), data.get('z')])
        
        return jsonify({'message': 'Position updated successfully'}), 200

    else:  # GET
        return jsonify(leader_position)
    
@app.route('/followers/<int:id>/position', methods=['POST'])
def update_follower_position(id):
    data = request.get_json()
    x = data.get('x')
    y = data.get('y')
    z = data.get('z')

    # Validate that all required data is present
    if None in (x, y, z):
        return jsonify({'error': 'Missing position data'}), 400
    
    if id not in {1,2,3}:
        return jsonify({'error': 'Follower ID not found'}), 404

    # Append the position to the respective follower's list
    match id:
        case 1:
            # follower 1
            follower_1_position.append([x, y, z])
        case 2:
            # follower 2
            follower_2_position.append([x, y, z])
        case 3:
            # follower 3
            follower_3_position.append([x, y, z])

    return jsonify({'message': f'Follower {id} position updated successfully'}), 200

if __name__ == '__main__':
    app.run(debug=True)