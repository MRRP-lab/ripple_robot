import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# Wave parameters
FIN_LENGTH = 330.0
AMPLITUDE = 0.3
TEMPORAL_FREQ = 0.5 * np.pi
SPATIAL_FREQ = 3 * np.pi / FIN_LENGTH

# Position arrays
x_positions = np.linspace(0, FIN_LENGTH, 800)
left_baseline = 1.0
right_baseline = -1.0

# Create figure and axes
fig, ax = plt.subplots(figsize=(8, 4))
ax.axhline(left_baseline, color="gray", linestyle="--", linewidth=1)
ax.axhline(right_baseline, color="gray", linestyle="--", linewidth=1)
ax.set(
    xlim=(0, FIN_LENGTH), 
    ylim=(-2, 2),
    xlabel="Longitudinal position (mm)",
    ylabel="Vertical displacement",
    title="Phase-Shifted Fin Gait Stability"
)

# Create plot elements
left_wave, = ax.plot([], [], linewidth=2, label="Left fin")
right_wave, = ax.plot([], [], linewidth=2, label="Right fin")
left_troughs, = ax.plot([], [], "ro", markersize=6, label="troughs")
right_troughs, = ax.plot([], [], "ro", markersize=6)
ax.legend(loc="upper right")


def find_troughs(wave_heights):
    """Find x-positions where wave has local minima."""
    is_trough = (wave_heights[1:-1] < wave_heights[:-2]) & (wave_heights[1:-1] < wave_heights[2:])
    trough_indices = np.where(is_trough)[0] + 1
    return x_positions[trough_indices]


def update_frame(frame_number):
    """Update animation for given frame."""
    time = frame_number * 0.05
    
    # Calculate wave heights
    left_heights = left_baseline + AMPLITUDE * np.sin(SPATIAL_FREQ * x_positions - TEMPORAL_FREQ * time)
    right_heights = right_baseline + AMPLITUDE * np.sin(SPATIAL_FREQ * x_positions - TEMPORAL_FREQ * time + np.pi)
    
    # Update waves
    left_wave.set_data(x_positions, left_heights)
    right_wave.set_data(x_positions, right_heights)
    
    # Update trough markers
    left_trough_x = find_troughs(left_heights)
    right_trough_x = find_troughs(right_heights)
    left_troughs.set_data(left_trough_x, [left_baseline] * len(left_trough_x))
    right_troughs.set_data(right_trough_x, [right_baseline] * len(right_trough_x))
    
    return left_wave, right_wave, left_troughs, right_troughs


# Create and save animation
total_frames = int(2 * np.pi / TEMPORAL_FREQ / 0.05)
animation = FuncAnimation(
    fig, 
    update_frame, 
    frames=total_frames,
    interval=40,
    blit=True
)

animation.save("out/gait_stability.gif", writer=PillowWriter(fps=20))
print("Saved as gait_stability.gif")