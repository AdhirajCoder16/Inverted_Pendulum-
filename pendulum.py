import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import control

# ====================== PARAMETERS ======================
M = 0.5      # cart mass
m = 0.2      # pendulum mass
b = 0.1      # cart friction
I = 0.006    # pendulum inertia
g = 9.81
l = 0.3      # length to pendulum center of mass

den = I*(M + m) + M*m*l**2

# State-Space Matrices
A = np.array([
    [0,      1,      0,           0],
    [0, -(I+m*l**2)*b/den,  (m**2*g*l**2)/den,   0],
    [0,      0,      0,           1],
    [0,   -(m*l*b)/den,     m*g*l*(M+m)/den,   0]
])

B = np.array([[0], [(I+m*l**2)/den], [0], [m*l/den]])

# LQR Controller
Q = np.diag([10, 1, 100, 1])   # High penalty on angle
R = np.array([[1]])
K, _, _ = control.lqr(A, B, Q, R)
print("LQR Gain K =", K.flatten())

# Closed-loop system
Acl = A - B @ K

# ====================== SIMULATION ======================
t = np.linspace(0, 8, 800)                    # 8 seconds, smooth animation
X0 = np.array([0, 0, 0.15, 0])                # Initial condition: 0.15 rad (~8.6°) tilt

# Simulate both systems
sys_open = control.ss(A, B, np.eye(4), 0)
sys_closed = control.ss(Acl, B, np.eye(4), 0)

_, x_open = control.initial_response(sys_open, T=t, X0=X0)
_, x_closed = control.initial_response(sys_closed, T=t, X0=X0)

# ====================== ANIMATION SETUP ======================
fig = plt.figure(figsize=(14, 7))
plt.suptitle('🔴 Inverted Pendulum Live Animation\nState-Space Model + LQR Controller', fontsize=16)

# Left: Open Loop (Unstable)
ax1 = fig.add_subplot(1, 2, 1)
ax1.set_xlim(-1.5, 1.5)
ax1.set_ylim(-1.2, 1.2)
ax1.set_title('Open Loop - Unstable (Falls)')
ax1.grid(True)

# Right: LQR Stabilized
ax2 = fig.add_subplot(1, 2, 2)
ax2.set_xlim(-1.5, 1.5)
ax2.set_ylim(-1.2, 1.2)
ax2.set_title('LQR Controller - Stabilized')
ax2.grid(True)

# Cart and Pendulum elements
def draw_pendulum(ax, x, theta, color='red', label=''):
    # Cart
    cart_width = 0.4
    cart_height = 0.2
    cart = plt.Rectangle((x - cart_width/2, -cart_height/2), cart_width, cart_height, 
                        fc=color, alpha=0.8, label=label)
    ax.add_patch(cart)
    
    # Pendulum rod
    px = x + l * np.sin(theta)
    py = l * np.cos(theta)
    line, = ax.plot([x, px], [0, py], color=color, lw=4)
    
    # Pendulum bob
    bob = plt.Circle((px, py), 0.08, fc=color)
    ax.add_patch(bob)
    
    return cart, line, bob

# Initialize elements
cart_open, line_open, bob_open = draw_pendulum(ax1, 0, 0, 'crimson', 'Unstable')
cart_closed, line_closed, bob_closed = draw_pendulum(ax2, 0, 0, 'blue', 'LQR')

# Text boxes for live values
info1 = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=10,
                bbox=dict(facecolor='white', alpha=0.8))
info2 = ax2.text(0.02, 0.95, '', transform=ax2.transAxes, fontsize=10,
                bbox=dict(facecolor='white', alpha=0.8))

# ====================== ANIMATION FUNCTION ======================
def animate(frame):
    # Open Loop (left)
    x_o = x_open[0, frame]
    theta_o = x_open[2, frame]
    force_o = 0  # No control
    
    cart_open.set_x(x_o - 0.2)
    line_open.set_data([x_o, x_o + l*np.sin(theta_o)], [0, l*np.cos(theta_o)])
    bob_open.center = (x_o + l*np.sin(theta_o), l*np.cos(theta_o))
    
    info1.set_text(f'Cart Pos: {x_o:.3f} m\nAngle: {np.rad2deg(theta_o):.1f}°\nForce: {force_o:.1f} N')
    
    # LQR Closed Loop (right)
    x_c = x_closed[0, frame]
    theta_c = x_closed[2, frame]
    force_c = (-K @ x_closed[:, frame])[0]
    
    cart_closed.set_x(x_c - 0.2)
    line_closed.set_data([x_c, x_c + l*np.sin(theta_c)], [0, l*np.cos(theta_c)])
    bob_closed.center = (x_c + l*np.sin(theta_c), l*np.cos(theta_c))
    
    info2.set_text(f'Cart Pos: {x_c:.3f} m\nAngle: {np.rad2deg(theta_c):.1f}°\nForce: {force_c:.1f} N')
    
    return (cart_open, line_open, bob_open, info1,
            cart_closed, line_closed, bob_closed, info2)

ani = FuncAnimation(fig, animate, frames=len(t), interval=30, blit=False, repeat=True)

plt.tight_layout()
plt.show()

# Optional: Save as GIF (uncomment when needed)
# ani.save('inverted_pendulum_live.gif', writer='pillow', fps=30)