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
    l: float = 0.2        # CoM distance [m]
    I: float = None       # inertia about pivot [kg*m^2]
    g: float = 9.81
    c: float = 0.02       # viscous damping [1/s]
    a_max: float = 20.0
    v_max: float = 1.0

    def __post_init__(self):
        if self.I is None:
            # default: point mass
            self.I = self.m * self.l**2


# ==========================================================
# PLANT
# ==========================================================

class InvertedPendulumPlant:

    def __init__(self, params: PendulumParameters):
        self.p = params

    """ Non-linear dynamics.
    Used to simulate the inverted pendulum.
    Input is the cart acceleration a. """
    def dynamics(self, state, a):
        # States are:
        # x: cart position
        # v: carte velocity
        # theta: pendulum angle (0 degree corresponds to upward position)
        # omega: pendulum angular velocity
        x, v, theta, omega = state
        p = self.p

        # Saturate acceleration
        # a = np.clip(a, -p.a_max, p.a_max)

        # Computes states at next time step
        dxdt = v # Change of position of the cart
        dvdt = a # Change of velocity of the cart
        dthetadt = omega # Angular change of the pendulum

        # Change of angular velocity of the pendulum
        domegadt = (
            (p.m * p.g * p.l / p.I) * np.sin(theta)
            - (p.m * p.l / p.I) * a * np.cos(theta)
            - p.c * omega
        )

        return np.array([dxdt, dvdt, dthetadt, domegadt])

    """ Linearization around upright theta=0.
    Returns the A,B,C,D matrices of the (linear) state space model. """
    def linear_matrices(self):
        p = self.p

        A = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, (p.m * p.g * p.l) / p.I, -p.c]
        ])

        B = np.array([
            [0],
            [1],
            [0],
            [-(p.m * p.l) / p.I]
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
        # Angle Error and derivative of the angle error
        theta = -state[2]
        omega = -state[3]

        self.integral += theta * self.dt

        return self.Kp*theta + self.Kd*omega + self.Ki*self.integral


class LQRController:

    def __init__(self, plant, Q, R):
        A, B, _, _ = plant.linear_matrices()
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P

    def __call__(self, t, state):
        return float(-self.K @ state)


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
        py = params.l * np.cos(theta[i])

        pend_line.set_data([cart_x, px], [0, py])

        return cart_patch, pend_line

    ani = animation.FuncAnimation(fig, update, frames=len(t), interval=10)
    return ani


def plot_snapshots(t, history, params, times):
    """Display the cart and pendulum at several time instants in a single figure."""
    x = history[:, 0]
    theta = history[:, 2]

    n = len(times)
    fig, axes = plt.subplots(1, n, figsize=(2.5 * n, 3))
    if n == 1:
        axes = [axes]

    cart_width = 0.1
    cart_height = 0.05
    margin = params.l + 0.1

    for ax, time in zip(axes, times):
        idx = np.argmin(np.abs(t - time))
        cart_x = x[idx]
        th = theta[idx]

        # Track
        ax.axhline(-cart_height / 2, color='gray', lw=1)

        # Cart
        ax.add_patch(plt.Rectangle(
            (cart_x - cart_width / 2, -cart_height / 2),
            cart_width, cart_height,
            color='steelblue'
        ))

        # Pendulum rod
        px = cart_x + params.l * np.sin(th)
        py = params.l * np.cos(th)
        ax.plot([cart_x, px], [0, py], 'k-', lw=2)

        ax.set_xlim(cart_x - margin, cart_x + margin)
        ax.set_ylim(-margin, margin)
        ax.set_aspect('equal')
        ax.set_xlabel(f"t = {time:.2f} s")
        ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
        for spine in ax.spines.values():
            spine.set_visible(False)

    fig.tight_layout()
    return fig


# ==========================================================
# EXAMPLE USAGE
# ==========================================================

if __name__ == "__main__":
    dt = 0.001

    params = PendulumParameters(m=0.3, l=0.25, c=0.1)
    plant = InvertedPendulumPlant(params)
    sim = Simulator(plant, dt=dt)

    # Initial states
    x0 = np.array([0.0, 0.0, np.deg2rad(5), 0.0])

    # --- choose controller ---
    # controller = lambda t,x: 0
    controller = PIDController(Kp=-100, Kd=-10, dt=dt)

    # For LQR:
    # Q = np.diag([1, 1, 100, 10])
    # R = np.array([[1]])
    # controller = LQRController(plant, Q, R)

    import os
    os.makedirs("figs", exist_ok=True)

    t, hist, u = sim.simulate(controller, x0, T=10)

    # Frequency Analysis
    # A, B, C, D = plant.linear_matrices()
    # # output = theta
    # C_theta = C[2,:]
    # D_theta = D[2,:]
    # # Linear plant
    # G = ctl.ss(A, B, C_theta, D_theta)

    # ctl.bode_plot(G, omega=2*np.pi*np.logspace(-1, 3, 1000), Hz=True, deg=True)
    # plt.savefig("figs/bode_plant.png")

    # # PID
    # s = ctl.tf('s')
    # Kp = -100
    # Kd = -10
    # Ki = 0
    # K = Kp + Ki/s + Kd*s

    # L = K*G
    # ctl.bode_plot(L, omega=2*np.pi*np.logspace(-1, 3, 1000), Hz=True, deg=True)
    # plt.savefig("figs/bode_loop.png")

    fig = plt.figure()
    plt.plot(t, hist[:,0]*1000)
    plt.ylabel("Cart position [mm]")
    plt.xlabel("Time [s]")
    plt.grid()
    fig.savefig("figs/cart_position.png")

    fig = plt.figure()
    plt.plot(t, hist[:,1]*1000)
    plt.ylabel("Cart Velocity [mm/s]")
    plt.xlabel("Time [s]")
    plt.grid()
    fig.savefig("figs/cart_velocity.png")

    fig = plt.figure()
    plt.plot(t, np.rad2deg(hist[:,2]))
    plt.ylabel("Pendulum angle [deg]")
    plt.xlabel("Time [s]")
    plt.grid()
    fig.savefig("figs/pendulum_angle.png")

    fig = plot_snapshots(t, hist, params, times=[0, 0.5, 1.0, 2.0, 5.0])
    fig.savefig("figs/snapshots.png")
