import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
# Parámetros del robot (longitudes, masas, etc.)
m1, m2 = 1.0, 1.0  # masas de los eslabones
l1, l2 = 1.0, 1.0  # longitudes de los eslabones
g = 9.81  # gravedad
# Matriz de inercia


def M(q):
    q1, q2 = q
    M11 = (m1 + m2) * l1**2 + m2 * l2**2 + 2 * m2 * l1 * l2 * np.cos(q2)
    M12 = m2 * l2**2 + m2 * l1 * l2 * np.cos(q2)
    M21 = M12
    M22 = m2 * l2**2
    return np.array([[M11, M12], [M21, M22]], dtype=np.float64)

# Matriz de Coriolis y centrífugas


def C(q, dq):
    q1, q2 = q
    dq1, dq2 = dq
    C11 = -m2 * l1 * l2 * np.sin(q2) * dq2
    C12 = -m2 * l1 * l2 * np.sin(q2) * (dq1 + dq2)
    C21 = m2 * l1 * l2 * np.sin(q2) * dq1
    C22 = 0
    return np.array([[C11, C12], [C21, C22]], dtype=np.float64)

# Torque gravitacional


def G(q):
    q1, q2 = q
    G1 = (m1 + m2) * g * l1 * np.cos(q1) + m2 * g * l2 * np.cos(q1 + q2)
    G2 = m2 * g * l2 * np.cos(q1 + q2)
    return np.array([G1, G2], dtype=np.float64)

# Controlador PID


def PID_control(q, dq, q_desired, dq_desired, integral_error, Kp, Ki, Kd, dt):
    error = q_desired - q
    d_error = dq_desired - dq
    integral_error += error * dt
    tau = Kp * error + Ki * integral_error + Kd * d_error
    return tau, integral_error

# Dinámica del sistema


def dynamics(t, state, q_desired, dq_desired, integral_error, Kp, Ki, Kd, dt):
    q = state[:2]
    dq = state[2:]
    tau, integral_error = PID_control(q, dq, q_desired, dq_desired, integral_error,
                                      Kp, Ki, Kd, dt)
    M_inv = np.linalg.inv(M(q))
    ddq = M_inv @ (tau - C(q, dq) @ dq - G(q))
    return np.hstack((dq, ddq)), integral_error


# Parámetros del controlador
Kp = np.array([50, 50], dtype=np.float64)
Ki = np.array([5, 5], dtype=np.float64)
Kd = np.array([10, 10], dtype=np.float64)
# Condiciones iniciales y tiempo
dt = 0.01
max_time = 25
time = np.arange(0, max_time, dt)
q0 = np.array([0, 0], dtype=np.float64)
dq0 = np.array([0, 0], dtype=np.float64)
state0 = np.hstack((q0, dq0))
q_desired = np.array([np.pi/4, np.pi/6], dtype=np.float64)
dq_desired = np.array([0, 0], dtype=np.float64)
integral_error = np.zeros(2, dtype=np.float64)
# Simulación
sol = solve_ivp(lambda t, y: dynamics(t, y, q_desired, dq_desired, integral_error,
                                      Kp, Ki, Kd, dt)[0],
                [0, max_time], state0, t_eval=time)
# Graficar resultados
plt.figure(figsize=(10, 5))
plt.plot(sol.t, sol.y[0], label='q1')
plt.plot(sol.t, sol.y[1], label='q2')
plt.plot(sol.t, np.full_like(sol.t, q_desired[0]), '--', label='q1 deseado',
         color='blue')
plt.plot(sol.t, np.full_like(sol.t, q_desired[1]), '--', label='q2 deseado',
         color='orange')
plt.xlabel('Tiempo [s]')
plt.ylabel('Ángulo [rad]')
plt.legend()
plt.title('Respuesta del controlador PID')
plt.show()
