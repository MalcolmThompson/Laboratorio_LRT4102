import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, step

# Tiempo de simulación
tiempo = np.linspace(0, 10, 1000)

# Parámetros del sistema (subamortiguado)
wn = 2
zeta = 0.2

# --- Controlador P ---
kp_p = 0.75
num_p = [kp_p * wn**2]
den_p = [1, 2 * zeta * wn, wn**2]
sistema_p = lti(num_p, den_p)
_, respuesta_p = step(sistema_p, T=tiempo)

# --- Controlador PD ---
kp_pd = 0.9
kd_pd = 0.12
num_pd = [kd_pd, kp_pd * wn**2]
den_pd = [1, 2 * zeta * wn + kd_pd, wn**2 + kp_pd]
sistema_pd = lti(num_pd, den_pd)
_, respuesta_pd = step(sistema_pd, T=tiempo)

# --- Controlador PID ---
kp_pid = 0.85
kd_pid = 0.09
ki_pid = 0.005
num_pid = [kd_pid, kp_pid, ki_pid]
den_pid = [1, 2 * zeta * wn + kd_pid, wn**2 + kp_pid, ki_pid]
sistema_pid = lti(num_pid, den_pid)
_, respuesta_pid = step(sistema_pid, T=tiempo)

# --- Crear subplots individuales ---
figura, ejes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# --- P ---
ejes[0].plot(tiempo, respuesta_p, color='purple', linewidth=2, label='Controlador P')
ejes[0].set_title('Respuesta con Controlador P')
ejes[0].set_ylabel('Salida')
ejes[0].legend()
ejes[0].grid(True)

# --- PD ---
ejes[1].plot(tiempo, respuesta_pd, color='orange', linewidth=2, label='Controlador PD')
ejes[1].set_title('Respuesta con Controlador PD')
ejes[1].set_ylabel('Salida')
ejes[1].legend()
ejes[1].grid(True)

# --- PID ---
ejes[2].plot(tiempo, respuesta_pid, color='cyan', linewidth=2, label='Controlador PID')
ejes[2].set_title('Respuesta con Controlador PID')
ejes[2].set_xlabel('Tiempo (s)')
ejes[2].set_ylabel('Salida')
ejes[2].legend()
ejes[2].grid(True)

# --- Mostrar ---
plt.tight_layout()
plt.show()
