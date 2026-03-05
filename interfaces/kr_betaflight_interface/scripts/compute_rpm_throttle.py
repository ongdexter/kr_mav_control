import pandas as pd
import numpy as np

df = pd.read_csv("FPVCycle_22.6mm(1920Kv)_Gemfan_Hurricane_51433(5.1x4.33x3)_PropGuard_4S.csv")
esc = df['ESC signal (µs)'].values
rpm = df['Motor Optical Speed (RPM)'].values

# Filter stable samples
mask = (rpm > 500) & (esc > 1050)
df_f = pd.DataFrame({'esc': esc[mask], 'rpm': rpm[mask]})

# Median RPM per throttle step
grouped = df_f.groupby('esc')['rpm'].median().reset_index()

# Fit
lin_coef  = np.polyfit(grouped['esc'], grouped['rpm'], 1)   # [a, b]
quad_coef = np.polyfit(grouped['esc'], grouped['rpm'], 2)   # [a, b, c]

print("Linear fit coefficients (a, b):", lin_coef)
print("Quadratic fit coefficients (a, b, c):", quad_coef)