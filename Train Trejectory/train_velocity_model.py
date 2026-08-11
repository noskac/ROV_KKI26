import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import joblib

# 1. Load Dataset
try:
    df = pd.read_csv('rov_velocity_dataset.csv')
    print("Dataset berhasil dimuat!")
except FileNotFoundError:
    print("Error: File rov_velocity_dataset.csv tidak ditemukan.")
    exit()

# 2. Persiapkan Data Fitur (X) dan Target (y)
# Fitur: Selisih PWM dari titik netral (1500). 
# Ini membantu model menangkap logika maju/mundur dengan lebih baik.
df['Delta_PWM'] = df['Avg_PWM'] - 1500
X = df[['Delta_PWM']].values
y = df['Velocity_m_s'].values

# 3. Model Fisika + ML (Polynomial Regression Degree 2)
# Kurva thruster bawah air biasanya proporsional dengan kuadrat kecepatan.
poly_features = PolynomialFeatures(degree=2, include_bias=False)
X_poly = poly_features.fit_transform(X)

model = LinearRegression()
model.fit(X_poly, y)

# 4. Evaluasi Model
y_pred = model.predict(X_poly)
r2 = r2_score(y, y_pred)
print(f"Akurasi Model (R2 Score): {r2:.4f}")
print(f"Koefisien Model: {model.coef_}")
print(f"Intercept Model: {model.intercept_}")

# 5. Export Model (Agar bisa dibaca oleh sistem Trajectory ROS 2 nantinya)
model_data = {
    'poly_transform': poly_features,
    'linear_model': model
}
joblib.dump(model_data, 'rov_kinematics_model.pkl')
print("Model diekspor ke: rov_kinematics_model.pkl")

# 6. Visualisasi Kurva (Opsional, untuk melihat stabilitas)
X_plot = np.linspace(df['Delta_PWM'].min() - 50, df['Delta_PWM'].max() + 50, 100).reshape(-1, 1)
X_plot_poly = poly_features.transform(X_plot)
y_plot = model.predict(X_plot_poly)

plt.figure(figsize=(8, 5))
plt.scatter(X, y, color='red', label='Data Kolam Aktual')
plt.plot(X_plot, y_plot, color='blue', label='Polynomial Regression Curve')
plt.title('ROV Thruster PWM vs Velocity')
plt.xlabel('Delta PWM (Avg_PWM - 1500)')
plt.ylabel('Velocity (m/s)')
plt.grid(True)
plt.legend()
plt.show()