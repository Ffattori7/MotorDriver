import pandas as pd
import matplotlib.pyplot as plt

filename = "logs/test_PD.csv"

data = pd.read_csv(
    filename,
    skiprows=4
)

# print(data.head())
# print(data.columns)

# exit()

# ---------------------------------------------------------
# PWM
# ---------------------------------------------------------

plt.figure()

plt.plot(
    data["time_ms"],
    data["pwm"],
    # label="Actual position [rev]"
)

plt.axhline(
    130.0,
    color="red",
    linestyle="--",
    label="PWM Saturation"
)

plt.xlabel("Time [ms]")
plt.ylabel("PWM")
plt.title("PWM Command [0-255]")
plt.grid()
plt.legend()
plt.tight_layout()

# ---------------------------------------------------------
# Velocity: reference vs measured
# ---------------------------------------------------------
plt.figure()

plt.plot(
    data["time_ms"],
    data["velocity_ref_rev_s"],
    label="Velocity reference [rev/s]"
)

plt.plot(
    data["time_ms"],
    data["velocity_rev_s"],
    label="Measured velocity [rev/s]"
)

plt.xlabel("Time [ms]")
plt.ylabel("Velocity [rev/s]")
plt.title("Velocity Tracking")
plt.grid()
plt.legend()
plt.tight_layout()


# ---------------------------------------------------------
# Position: reference vs measured
# ---------------------------------------------------------
plt.figure()

plt.plot(
    data["time_ms"],
    data["position_ref_rev"]*(2*3.14)/4.4*(20*2)/(2*3.14),
    label="Position reference [rev]"
)

plt.plot(
    data["time_ms"],
    data["position_rev"]*(2*3.14)/4.4*(20*2)/(2*3.14),
    label="Measured position [rev]"
)

plt.xlabel("Time [ms]")
plt.ylabel("Position [rev]")
plt.title("Position Tracking")
plt.grid()
plt.legend()
plt.tight_layout()

# ---------------------------------------------------------
# Current: instantaneous vs maximum
# ---------------------------------------------------------

plt.figure()

plt.plot(
    data["time_ms"],
    data["current_A"],
    label="Motor current [A]"
)

plt.plot(
    data["time_ms"],
    data["maxCurrent_A"],
    linestyle="--",
    label="Maximum current [A]"
)

plt.axhline(
    5.0,
    color="red",
    linestyle="--",
    label="Current limit [A]"
)

plt.xlabel("Time [ms]")
plt.ylabel("Current [A]")
plt.title("Motor Current")
plt.grid()
plt.legend()
plt.tight_layout()

# # Theoretical control-loop period
# THEORETICAL_PERIOD_US = 20_000

# # ---------------------------------------------------------
# # Loop timing
# # ---------------------------------------------------------
# plt.figure()

# plt.plot(
#     data["time_ms"],
#     data["loop_period_us"],
#     label="Actual loop period"
# )

# plt.axhline(
#     THEORETICAL_PERIOD_US,
#     linestyle="--",
#     label="Theoretical period (20 ms)"
# )

# plt.plot(
#     data["time_ms"],
#     data["loop_exe_us"],
#     label="Loop execution time"
# )

# plt.xlabel("Time [ms]")
# plt.ylabel("Time [µs]")
# plt.title("Control Loop Timing")
# plt.grid()
# plt.legend()
# plt.tight_layout()

# # ---------------------------------------------------------
# # Loop timing margin
# # ---------------------------------------------------------
# timing_margin = THEORETICAL_PERIOD_US - data["loop_exe_us"]

# plt.figure()

# plt.plot(
#     data["time_ms"],
#     timing_margin,
#     label="Timing margin"
# )

# plt.axhline(
#     0,
#     linestyle="--",
#     label="Zero margin"
# )

# plt.xlabel("Time [ms]")
# plt.ylabel("Available time [µs]")
# plt.title("Control Loop Timing Margin")
# plt.grid()
# plt.legend()
# plt.tight_layout()

# # ---------------------------------------------------------
# # Loop timing stats
# # ---------------------------------------------------------
# print("Loop timing statistics:")

# print(f"Theoretical period: {THEORETICAL_PERIOD_US} µs")
# print(f"Mean actual period: {data['loop_period_us'].mean():.1f} µs")
# print(f"Min actual period:  {data['loop_period_us'].min():.1f} µs")
# print(f"Max actual period:  {data['loop_period_us'].max():.1f} µs")

# print(f"Mean execution:     {data['loop_exe_us'].mean():.1f} µs")
# print(f"Max execution:      {data['loop_exe_us'].max():.1f} µs")

plt.show()