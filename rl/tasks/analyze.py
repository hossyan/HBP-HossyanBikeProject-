import numpy as np
import sys

a = np.genfromtxt(sys.argv[1], delimiter=',', encoding='latin-1')
a = a[~np.isnan(a).any(axis=1)]
t, th, v, cmd = a[:,0], a[:,1], a[:,2], a[:,3]

ch = np.where(np.diff(cmd) != 0)[0]
t0 = t[ch[0] + 1] if len(ch) else t[0]
m = t >= t0 + 2000.0
th, v = th[m], v[m]

theta_rotor = th * 7.75
A = np.column_stack([np.ones_like(v), np.cos(14*theta_rotor), np.sin(14*theta_rotor)])
c, *_ = np.linalg.lstsq(A, v, rcond=None)

print(f"samples   = {len(v)}")
print(f"rotor rev = {(theta_rotor.max()-theta_rotor.min())/2/np.pi:.2f}")
print(f"mean = {c[0]:.4f} rad/s")
print(f"k=14 amplitude = {np.hypot(c[1], c[2]):.4f} rad/s")
print(f"raw std = {v.std():.4f}")
