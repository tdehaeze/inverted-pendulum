import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from dataclasses import dataclass
from scipy.linalg import solve_continuous_are
import matplotlib.animation as animation
import control as ctl

# ==========================================================
# PARAMETERS
# ==========================================================

@dataclass
class PendulumParameters:
    m: float = 0.2        # pendulum mass [kg]
    l: float = 0.2        # pivot-to-CoM distance (half total length) [m]
    I: float = None       # inertia about CoM [kg*m^2]
    g: float = 9.81
    c: float = 0.02       # angular damping coefficient [N*m*s/rad]
    a_max: float = 20.0
    v_max: float = 1.0

    def __post_init__(self):
        if self.I is None:
            # uniform rod: I_com = 1/12 * m * (2*l)^2
            self.I = (1/12) * self.m * (2*self.l)**2

# ==========================================================
# PLANT
# ==========================================================

class InvertedPendulumPlant:

    def __init__(self, params: PendulumParameters):
        self.p = params

    """ Non-linear dynamics.
    Used to simulate the inverted pendulum.
    Input u is the cart linear acceleration (x_dot_dot).
    Equation: (I + m*l^2)*theta_ddot + c*theta_dot + m*g*l*sin(theta) = -m*l*u*cos(theta) """
    def dynamics(self, state, u):
        # States are:
        # x:     cart position [m]
        # v:     cart velocity [m/s]
        # theta: pendulum angle [rad]  (theta=0: downward, theta=pi: upright)
        # omega: pendulum angular velocity [rad/s]
        x, v, theta, omega = state
        p = self.p

        dxdt     = v
        dvdt     = u
        dthetadt = omega

        D = p.I + p.m * p.l**2  # effective inertia about pivot
        domegadt = (-p.c * omega - p.m * p.g * p.l * np.sin(theta) - p.m * p.l * u * np.cos(theta)) / D

        return np.array([dxdt, dvdt, dthetadt, domegadt])

    """ Linearization around the upright position (phi = 0, where phi = theta - pi).
    Equation: (I + m*l^2)*phi_ddot + c*phi_dot - m*g*l*phi = m*l*u
    States: [x, x_dot, phi, phi_dot]
    Returns the A, B, C, D matrices of the linear state space model. """
    def linear_matrices(self):
        p = self.p

        D = p.I + p.m * p.l**2  # effective inertia about pivot

        A = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, (p.m * p.g * p.l) / D, -p.c / D]
        ])

        B = np.array([
            [0],
            [1],
            [0],
            [(p.m * p.l) / D]
        ])

        C = np.array([
            [1, 0, 0, 0],  # cart position
            [0, 1, 0, 0],  # cart velocity
            [0, 0, 1, 0]   # pendulum angle
        ])

        D = np.array([
            [0],
            [0],
            [0]
        ])

        return A, B, C, D


# ==========================================================
# SIMULATOR
# ==========================================================

class Simulator:

    def __init__(self, plant: InvertedPendulumPlant, dt=0.001):
        self.plant = plant
        self.dt = dt

    """ Simulation of the closed-loop system:
        - controller: Controller used for the simulations
        - x0: Initial states
        - T: Simulation time
        Returns:
        - t: Vector of time
        - history: Simulated states
        - u_hist: Controller output
    """
    def simulate(self, controller, x0, T):

        N = int(T / self.dt) # Number of points to be simulated
        t = np.linspace(0, T, N) # Time vector [s]

        x = x0.copy()
        history = np.zeros((N, len(x0)))
        u_hist = np.zeros(N)

        for k in range(N):

            u = controller(t[k], x)
            u_hist[k] = u

            dx = self.plant.dynamics(x, u)
            x = x + dx * self.dt

            # velocity saturation
            # x[1] = np.clip(x[1], -self.plant.p.v_max, self.plant.p.v_max)

            history[k] = x

        return t, history, u_hist


# ==========================================================
# CONTROLLERS
# ==========================================================

class PIDController:

    def __init__(self, Kp=0, Kd=0, Ki=0, dt=0.001):
        self.Kp = Kp # Proportional gain
        self.Kd = Kd # Derivative gain
        self.Ki = Ki # Integrator gain
        self.integral = 0 # Keep track of the integrator value
        self.dt = dt

    def __call__(self, t, state):
        phi     = state[2] - np.pi  # deviation from upright
        phi_dot = state[3]

        self.integral += phi * self.dt

        return self.Kp*phi + self.Kd*phi_dot + self.Ki*self.integral


class LQRController:

    def __init__(self, plant, Q, R):
        A, B, _, _ = plant.linear_matrices()
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P

    def __call__(self, t, state):
        phi_state = state.copy()
        phi_state[2] = state[2] - np.pi  # phi = theta - pi
        return float(-self.K @ phi_state)


# ==========================================================
# VISUALIZATION
# ==========================================================

def animate(t, history, params):

    x = history[:,0]
    theta = history[:,2]

    fig, ax = plt.subplots()
    ax.set_xlim(-1, 1)
    ax.set_ylim(-0.5, 0.5)

    cart_width = 0.1
    cart_height = 0.05

    cart_patch = plt.Rectangle((0,0), cart_width, cart_height)
    pend_line, = ax.plot([], [], lw=2)

    ax.add_patch(cart_patch)

    def update(i):

        cart_x = x[i]
        cart_patch.set_xy((cart_x - cart_width/2, -cart_height/2))

        px = cart_x + params.l * np.sin(theta[i])
        py = -params.l * np.cos(theta[i])

        pend_line.set_data([cart_x, px], [0, py])

        return cart_patch, pend_line

    ani = animation.FuncAnimation(fig, update, frames=len(t), interval=10)
    return ani


def plot_snapshots(t, history, params, times):
    """Display cart and pendulum at several time instants on a shared figure.
    The horizontal axis corresponds to the physical cart position x.
    Snapshots at the same position are superimposed."""
    x_hist = history[:, 0]
    theta_hist = history[:, 2]

    cart_width = 0.1
    cart_height = 0.05

    indices  = [np.argmin(np.abs(t - time)) for time in times]
    cart_xs  = [x_hist[i] for i in indices]

    # x range: actual cart travel + one pendulum length of margin on each side
    x_lo = min(cart_xs) - params.l - cart_width
    x_hi = max(cart_xs) + params.l + cart_width
    if x_hi - x_lo < 4 * params.l:          # ensure minimum width
        x_mid = (x_lo + x_hi) / 2
        x_lo, x_hi = x_mid - 2 * params.l, x_mid + 2 * params.l

    y_lo = -(cart_height / 2 + 0.12)        # room for time labels
    y_hi = params.l + 0.05

    # Size the figure so the aspect ratio is physically correct
    y_span = y_hi - y_lo
    x_span = x_hi - x_lo
    fig_h  = 3.5
    fig_w  = fig_h * x_span / y_span + 0.8  # +0.8 for margins
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    colors = plt.cm.plasma(np.linspace(0.1, 0.9, len(times)))

    # Track
    ax.axhline(-cart_height / 2, color='gray', lw=1, zorder=0)

    for color, time, idx, cart_x in zip(colors, times, indices, cart_xs):
        th = theta_hist[idx]

        # Cart
        ax.add_patch(plt.Rectangle(
            (cart_x - cart_width / 2, -cart_height / 2),
            cart_width, cart_height,
            color=color, alpha=0.85, zorder=2
        ))

        # Pendulum rod
        px = cart_x + params.l * np.sin(th)
        py = -params.l * np.cos(th)
        ax.plot([cart_x, px], [0, py], '-', color=color, lw=2, zorder=3)

        # Time label just below the cart
        ax.text(cart_x, -cart_height / 2 - 0.01, f"t = {time:.2f} s",
                ha='center', va='top', fontsize=8, color=color)

    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_aspect('equal')
    ax.set_xlabel("x [m]")
    ax.tick_params(left=False, labelleft=False)
    for spine in ['left', 'top', 'right']:
        ax.spines[spine].set_visible(False)

    fig.tight_layout()
    return fig


# ==========================================================
# EXAMPLE USAGE
# ==========================================================

if __name__ == "__main__":
    import os
    dt = 0.001
    os.makedirs("figs", exist_ok=True)

    # Fitted parameters from free-response measurement
    params = PendulumParameters(m=0.03, l=0.15, c=0.00032)
    plant  = InvertedPendulumPlant(params)
    sim    = Simulator(plant, dt=dt)

    # Critical proportional gain (stability threshold)
    # Derived from Routh-Hurwitz on linearised closed-loop:
    #   kp_crit = c² / (4·m·l·(I + m·l²)) + g
    m, l, c = params.m, params.l, params.c
    D       = params.I + m * l**2
    kp_crit = c**2 / (4 * m * l * D) + params.g
    print(f"kp_critical = {kp_crit:.4f}")

    # Initial state: 5° from upright
    x0 = np.array([0.0, 0.0, np.pi + np.deg2rad(5), 0.0])

    for name, kp in [("kp_below", 0.8 * kp_crit),
                     ("kp_above", 1.2 * kp_crit)]:

        print(f"{name}: kp = {kp:.4f}")
        controller = PIDController(Kp=-kp, Kd=0, Ki=0, dt=dt)
        t, hist, u_hist = sim.simulate(controller, x0, T=10)

        phi_deg  = np.rad2deg(hist[:, 2] - np.pi)
        cart_mm  = hist[:, 0] * 1000

        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        axes[0].plot(t, phi_deg)
        axes[0].axhline(0, color='k', lw=0.5, ls='--')
        axes[0].set_ylabel("Pendulum angle φ [deg]")
        axes[0].set_title(f"P controller — kp = {kp:.3f}  "
                          f"({'below' if 'below' in name else 'above'} "
                          f"critical = {kp_crit:.3f})")
        axes[0].grid(True)

        axes[1].plot(t, cart_mm)
        axes[1].set_ylabel("Cart position [mm]")
        axes[1].set_xlabel("Time [s]")
        axes[1].grid(True)

        fig.tight_layout()
        fig.savefig(f"figs/{name}.png", dpi=150)
        plt.close(fig)
        print(f"  saved figs/{name}.png")

    # PD controller: kp = 1.2*kp_crit, kd chosen for critical damping
    # From b² = 4·a·k on the closed-loop characteristic equation:
    #   kd = 1/(m·l) · (2·sqrt(m·l·kp - m·g·l)·sqrt(I + m·l²) - c)
    kp = 1.2 * kp_crit
    kd = (1 / (m * l)) * (2 * np.sqrt(m * l * kp - m * params.g * l) * np.sqrt(D) - c)
    print(f"kp_kd: kp = {kp:.4f}, kd = {kd:.4f}")

    controller = PIDController(Kp=-kp, Kd=-kd, Ki=0, dt=dt)
    t, hist, u_hist = sim.simulate(controller, x0, T=10)

    phi_deg = np.rad2deg(hist[:, 2] - np.pi)
    cart_mm = hist[:, 0] * 1000

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    axes[0].plot(t, phi_deg)
    axes[0].axhline(0, color='k', lw=0.5, ls='--')
    axes[0].set_ylabel("Pendulum angle φ [deg]")
    axes[0].set_title(f"PD controller — kp = {kp:.3f}, kd = {kd:.3f}  (critically damped)")
    axes[0].grid(True)

    axes[1].plot(t, cart_mm)
    axes[1].set_ylabel("Cart position [mm]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True)

    fig.tight_layout()
    fig.savefig("figs/kp_kd.png", dpi=150)
    plt.close(fig)
    print("  saved figs/kp_kd.png")
