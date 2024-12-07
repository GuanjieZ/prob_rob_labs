import numpy as np
import plotly.graph_objects as go
import pandas as pd
import os
from pathlib import Path

current_dir = Path(__file__).resolve()
data_path = current_dir.parent.parent

# Read the data from the text file
file_name = str(data_path)+"/labs/datalog/red.txt"
data = np.loadtxt(file_name, delimiter=",")

variances = np.var(data, axis=0)
for i, var in enumerate(variances):
    print(f"Variance of column {i+1}: {var:.4f}")
# Create a plotly figure

fig = go.Figure()

# Add traces for each column
for i in range(data.shape[1]):
    if i % 2 == 0:
        axis = 'x'
    elif i % 2 == 1:
        axis = 'y'
    fig.add_trace(go.Scatter(
        y=data[:, i],
        mode='lines',
        name=f'{axis}_{i//2+1}'
    ))

# Update layout for better visualization
fig.update_layout(
    title="Cyan",
    xaxis_title="Index",
    yaxis_title="Value",
    legend_title="Columns",
    template="plotly"
)

# Show the plot
fig.show()

