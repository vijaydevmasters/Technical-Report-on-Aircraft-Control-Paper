import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, lsim

# State Space matrices
A = np.array([[-0.0558, -0.9968, 0.0802, 0.0415],
              [0.5980, -0.1150, -0.0318, 0],
              [-3.0500, 0.3880, -0.4650, 0],
              [0, 0.0805, 1, 0]])
B = np.array([[0.0729, 0],
              [-4.75, 0.00775],
              [0.15300, 0.1430],
              [0, 0]])
C = np.array([[1, 0, 0, 0],
              [0, 0, 0, 1]])
D = np.array([[0, 0],
              [0, 0]])

# Define system
sys = lti(A, B, C, D)

# Process and Measurement noise covariance
Qn = 2.3
Rn = 1

# Time vector
t = np.arange(0, 10.1, 0.1)  # Time vector

# Sinusoidal input
u = np.column_stack((np.sin(t), np.zeros_like(t)))

# Simulate the true system
_, x_true, _ = lsim(sys, u, t)

# Add noise to measurements (for simulation)
y_measured = lsim(sys, u, t)[1] + np.sqrt(Rn) * np.random.randn(len(t), C.shape[0])

state = ["Side Slip Angle", "Roll Angle"]
# Kalman filter simulation.
# Plotting

for i in range(x_true.shape[1]):
    plt.subplot(x_true.shape[1], 1, i + 1)
    plt.plot(t, x_true[:, i], 'b', label='True')
    plt.plot(t, y_measured[:, i], 'g--', label='Measured')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel(state[i])
    plt.title('Kalman Filter Response')

plt.tight_layout()
plt.show()