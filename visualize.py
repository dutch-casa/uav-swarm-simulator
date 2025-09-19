#!/usr/bin/env python3
"""
UAV Swarm Visualization Script

Generates visualizations from simulation trace data and metrics.
Creates animated plots showing agent movements and final statistics.
"""

import sys
import json
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from pathlib import Path
import argparse

def load_map(map_file):
    """Load map file to get obstacles."""
    try:
        with open(map_file, 'r') as f:
            lines = f.readlines()

        # Filter out comments and empty lines
        map_lines = []
        for line in lines:
            line = line.strip()
            if line and not line.startswith('//'):
                map_lines.append(line)

        if not map_lines:
            return None

        # Convert to 2D array
        obstacles = []
        for y, line in enumerate(map_lines):
            for x, char in enumerate(line):
                if char == '#':
                    obstacles.append((x, y))

        # Get grid dimensions
        width = max(len(line) for line in map_lines) if map_lines else 0
        height = len(map_lines)

        return {
            'obstacles': obstacles,
            'width': width,
            'height': height,
            'grid': map_lines
        }
    except Exception as e:
        print(f"Warning: Could not load map file {map_file}: {e}")
        return None

def load_data(trace_file, metrics_file, map_file=None):
    """Load trace CSV and metrics JSON files."""
    # Load trace data
    try:
        trace_df = pd.read_csv(trace_file)
        print(f"Loaded {len(trace_df)} trace records")
    except Exception as e:
        print(f"Error loading trace file: {e}")
        return None, None, None

    # Load metrics data
    try:
        with open(metrics_file, 'r') as f:
            metrics = json.load(f)
        print(f"Loaded metrics: {metrics}")
    except Exception as e:
        print(f"Error loading metrics file: {e}")
        return trace_df, None, None

    # Load map data if provided
    map_data = None
    if map_file:
        map_data = load_map(map_file)
        if map_data:
            print(f"Loaded map: {map_data['width']}x{map_data['height']} with {len(map_data['obstacles'])} obstacles")

    return trace_df, metrics, map_data

def create_static_visualization(trace_df, metrics, output_file, map_data=None):
    """Create a static visualization showing final paths and metrics."""

    # Set up the plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # Plot 1: Final agent paths
    ax1.set_title('UAV Swarm Final Paths', fontsize=14, fontweight='bold')

    # Get grid dimensions
    max_x = trace_df['x'].max()
    max_y = trace_df['y'].max()

    # Use map dimensions if available
    if map_data:
        max_x = max(max_x, map_data['width'] - 1)
        max_y = max(max_y, map_data['height'] - 1)

    # Draw grid
    for i in range(max_x + 2):
        ax1.axvline(x=i-0.5, color='lightgray', linewidth=0.5)
    for i in range(max_y + 2):
        ax1.axhline(y=i-0.5, color='lightgray', linewidth=0.5)

    # Draw obstacles first (so they appear behind paths)
    if map_data and map_data['obstacles']:
        obstacle_x = [obs[0] for obs in map_data['obstacles']]
        obstacle_y = [obs[1] for obs in map_data['obstacles']]
        ax1.scatter(obstacle_x, obstacle_y,
                   color='brown', s=400, marker='s',
                   alpha=0.8, label='Obstacles', zorder=1)
        print(f"Drew {len(map_data['obstacles'])} obstacles")

    # Plot agent paths
    agents = trace_df['agent_id'].unique()
    colors = plt.cm.tab10(np.linspace(0, 1, len(agents)))

    for i, agent in enumerate(agents):
        agent_data = trace_df[trace_df['agent_id'] == agent].sort_values('tick')

        # Plot path
        ax1.plot(agent_data['x'], agent_data['y'],
                color=colors[i], linewidth=2, alpha=0.7,
                label=f'Agent {agent[:8]}...')

        # Mark start and end
        if len(agent_data) > 0:
            start_pos = agent_data.iloc[0]
            end_pos = agent_data.iloc[-1]

            ax1.scatter(start_pos['x'], start_pos['y'],
                       color=colors[i], s=100, marker='o',
                       edgecolors='black', linewidth=2)
            ax1.scatter(end_pos['x'], end_pos['y'],
                       color=colors[i], s=100, marker='s',
                       edgecolors='black', linewidth=2)

    ax1.set_xlim(-0.5, max_x + 0.5)
    ax1.set_ylim(-0.5, max_y + 0.5)
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax1.grid(True, alpha=0.3)

    # Plot 2: Metrics summary
    ax2.axis('off')
    ax2.set_title('Simulation Metrics', fontsize=14, fontweight='bold')

    if metrics:
        # Create metrics text
        metrics_text = f"""
        🎯 Mission Summary:
        ├─ Total Agents: {len(agents)}
        ├─ Simulation Ticks: {metrics.get('makespan', 'N/A')}
        ├─ Wall Time: {metrics.get('wall_time_ms', 'N/A')} ms

        📡 Communication:
        ├─ Total Messages: {metrics.get('total_messages', 'N/A')}
        ├─ Dropped Messages: {metrics.get('dropped_messages', 'N/A')}
        ├─ Drop Rate: {metrics.get('drop_rate', 'N/A'):.2%}

        🔄 Planning:
        ├─ Total Replans: {metrics.get('total_replans', 'N/A')}
        ├─ Avg Replans/Agent: {metrics.get('total_replans', 0) / len(agents):.1f}

        ⚠️ Safety:
        └─ Collisions: {'❌ YES' if metrics.get('collision_detected', False) else '✅ None'}
        """

        ax2.text(0.1, 0.9, metrics_text, transform=ax2.transAxes,
                fontsize=12, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))

        # Add performance indicators
        efficiency_score = 100 * (1 - metrics.get('drop_rate', 0))
        collision_penalty = 50 if metrics.get('collision_detected', False) else 0
        overall_score = max(0, efficiency_score - collision_penalty)

        score_text = f"Overall Performance: {overall_score:.0f}/100"
        score_color = 'green' if overall_score >= 80 else 'orange' if overall_score >= 60 else 'red'

        ax2.text(0.1, 0.1, score_text, transform=ax2.transAxes,
                fontsize=14, fontweight='bold', color=score_color,
                bbox=dict(boxstyle="round,pad=0.3", facecolor=score_color, alpha=0.2))
    else:
        ax2.text(0.5, 0.5, 'No metrics available', transform=ax2.transAxes,
                ha='center', va='center', fontsize=14)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Static visualization saved to: {output_file}")

    return fig

def create_animated_visualization(trace_df, metrics, output_file):
    """Create an animated GIF showing agent movements over time."""

    # Set up the plot
    fig, ax = plt.subplots(figsize=(10, 8))

    # Get data bounds
    max_x = trace_df['x'].max()
    max_y = trace_df['y'].max()
    max_tick = trace_df['tick'].max()

    # Draw grid
    for i in range(max_x + 1):
        ax.axvline(x=i, color='lightgray', linewidth=0.5)
    for i in range(max_y + 1):
        ax.axhline(y=i, color='lightgray', linewidth=0.5)

    ax.set_xlim(-0.5, max_x + 0.5)
    ax.set_ylim(-0.5, max_y + 0.5)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.grid(True, alpha=0.3)

    # Prepare agent data
    agents = trace_df['agent_id'].unique()
    colors = plt.cm.tab10(np.linspace(0, 1, len(agents)))
    agent_colors = {agent: colors[i] for i, agent in enumerate(agents)}

    # Animation function
    def animate(frame):
        ax.clear()

        # Redraw grid
        for i in range(max_x + 1):
            ax.axvline(x=i, color='lightgray', linewidth=0.5)
        for i in range(max_y + 1):
            ax.axhline(y=i, color='lightgray', linewidth=0.5)

        ax.set_xlim(-0.5, max_x + 0.5)
        ax.set_ylim(-0.5, max_y + 0.5)
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.grid(True, alpha=0.3)

        # Current tick
        current_tick = frame
        ax.set_title(f'UAV Swarm Simulation - Tick {current_tick}/{max_tick}',
                    fontsize=14, fontweight='bold')

        # Plot agent trails and current positions
        for agent in agents:
            agent_data = trace_df[trace_df['agent_id'] == agent]

            # Trail (up to current tick)
            trail_data = agent_data[agent_data['tick'] <= current_tick].sort_values('tick')
            if len(trail_data) > 1:
                ax.plot(trail_data['x'], trail_data['y'],
                       color=agent_colors[agent], linewidth=1, alpha=0.5)

            # Current position
            current_data = agent_data[agent_data['tick'] == current_tick]
            if len(current_data) > 0:
                pos = current_data.iloc[0]
                ax.scatter(pos['x'], pos['y'],
                          color=agent_colors[agent], s=100,
                          edgecolors='black', linewidth=2, zorder=5)

    # Create animation
    anim = animation.FuncAnimation(fig, animate, frames=max_tick+1,
                                  interval=200, repeat=True)

    # Save as GIF
    gif_file = output_file.replace('.png', '.gif')
    anim.save(gif_file, writer='pillow', fps=5)
    print(f"Animated visualization saved to: {gif_file}")

    return anim

def main():
    parser = argparse.ArgumentParser(description='Visualize UAV swarm simulation results')
    parser.add_argument('trace_file', help='Path to trace CSV file')
    parser.add_argument('metrics_file', help='Path to metrics JSON file')
    parser.add_argument('output_file', help='Output image file path')
    parser.add_argument('--animate', action='store_true', help='Create animated GIF')

    args = parser.parse_args()

    # Load data
    trace_df, metrics = load_data(args.trace_file, args.metrics_file)
    if trace_df is None:
        sys.exit(1)

    # Create visualization
    try:
        fig = create_static_visualization(trace_df, metrics, args.output_file)

        if args.animate and len(trace_df) > 0:
            create_animated_visualization(trace_df, metrics, args.output_file)

        print("Visualization complete!")

    except Exception as e:
        print(f"Error creating visualization: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()