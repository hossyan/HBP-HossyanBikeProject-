import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib import cm

# --- 物理パラメータ (48V / モータ4個) ---
MA = 12.0
MB = 8.0
g = 9.81
Kt = 0.09549
# Kt = 0.070175
R = 0.055
V = 32.0
eta = 0.5
Imax = 50.0
Im = 0.0001
num_motors = 4

THETA_START_DEG = 5.0
theta_start = np.radians(THETA_START_DEG)

def get_dynamics_values(theta, d_theta, G, l_val):
    i = np.clip((V - Kt * G * d_theta) / R, 0, Imax)
    cos_half = np.clip(np.cos(theta / 2), 0.01, 1.0)
    sin_half = np.sin(theta / 2)
    
    total_torque = G * eta * (num_motors * Kt * i)
    total_inertia = (num_motors * Im) * (G**2) * eta
    
    # 加速度 d2_theta (ノートの最終式)
    num = (total_torque / (l_val * cos_half)) - MA * g + (MA * l_val * (d_theta**2) * sin_half / 2)
    den = MA * l_val * cos_half + (total_inertia / (l_val * cos_half))
    d2_theta = num / den
    
    # 地面を蹴る力 f
    # 分子の (Kt*i - Im*d2_theta*G) の部分は、モータ1個あたりの実質出力トルク
    net_torque_per_motor = Kt * i - Im * d2_theta * G
    f = (G * eta * num_motors * net_torque_per_motor) / (l_val * cos_half)
    
    return d2_theta, f

def system_dynamics(t, state, G, l_val):
    theta, d_theta = state
    d2_theta, _ = get_dynamics_values(theta, d_theta, G, l_val)
    return [d_theta, d2_theta]

def takeoff_event(t, state, G, l_val):
    theta, d_theta = state
    _, f = get_dynamics_values(theta, d_theta, G, l_val)
    
    # 判定式: (蹴る力) - (全体の重さ)
    return f - (MA + MB) * g

takeoff_event.terminal = True  # イベント発生で停止
takeoff_event.direction = -1   # 正から負に変わる時だけ検知

def calc_height(l_val, G):
    # 初期状態でそもそも浮かない（f < 重力）場合はジャンプ不可
    _, f_initial = get_dynamics_values(theta_start, 0, G, l_val)
    if f_initial <= (MA + MB) * g:
        return 0.0

    sol = solve_ivp(
        system_dynamics, [0, 0.5], [theta_start, 0], args=(G, l_val), 
        events=[takeoff_event], max_step=0.001
    )
    
    if sol.t_events[0].size > 0:
        theta_f, d_theta_f = sol.y[0, -1], sol.y[1, -1]
        # 離陸時の垂直速度 (ノートの式⑧)
        v_takeoff = l_val * d_theta_f * np.cos(theta_f / 2)
        # ジャンプ高さ (式②)
        h = (1 / (2 * g)) * (MA / (MA + MB)) * (v_takeoff**2)
        return h if h < 10.0 else 0.0
    return 0.0

# --- 計算と描画 ---
l_list = np.linspace(0.5, 0.8, 100)
G_list = np.linspace(10, 70, 100)
X, Y = np.meshgrid(l_list, G_list)
Z = np.vectorize(calc_height)(X, Y)

# --- 描画のブラッシュアップ ---
fig = plt.figure(figsize=(14, 9))
ax = fig.add_subplot(111, projection='3d')

# カラーマップを 'turbo' (青→緑→赤) または 'magma' (黒→紫→オレンジ→白) に変更
# antialiased=True と edgecolor='none' で滑らかさを維持
surf = ax.plot_surface(X, Y, Z, cmap='turbo', 
                       antialiased=True, 
                       edgecolor='none', 
                       rcount=100, ccount=100, 
                       alpha=0.9)

# 右側にカラーバーを追加
cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=15, pad=0.1)
cbar.set_label('Jump Height $h$ [m]', fontsize=12)

# タイトルとラベル
ax.set_title(f'Optimized Jump Height (Force-based)\n$V$={V}V, Motors={num_motors}', fontsize=15, pad=20)
ax.set_xlabel('Link Length $l$ [m]', fontsize=12, labelpad=10)
ax.set_ylabel('Gear Ratio $G$', fontsize=12, labelpad=10)
ax.set_zlabel('Height $h$ [m]', fontsize=12, labelpad=10)

# 視点の微調整（山が見えやすい角度）
ax.view_init(elev=30, azim=210)

plt.tight_layout()
plt.show()