Web VPython 3.2
from vpython import *

# Winkel in Radiant
beta = 45 * pi/180
theta = beta / 2

scene = canvas(title="VL53L5CX FoV – mehrere Distanzen", background=color.white)

# Sensor
box(pos=vector(0,0,0), size=vector(0.05,0.05,0.02), color=color.red)

# Achsen
arrow(pos=vector(0,0,0), axis=vector(0.3,0,0), color=color.red)
arrow(pos=vector(0,0,0), axis=vector(0,0.3,0), color=color.green)
arrow(pos=vector(0,0,0), axis=vector(0,0,0.3), color=color.blue)

def make_plane(d, col):
    half_size = d * tan(theta)
    return box(pos=vector(0,0,d),
               size=vector(2*half_size, 2*half_size, 0.01),
               color=col,
               opacity=0.3)

# Distanzen
distances = [
    (0.5, color.blue),
    (1.0, color.green),
    (1.5, color.orange)
]

# Ray-Funktion
def ray_point(ax, ay, d):
    tx = tan(ax)
    ty = tan(ay)
    dir = vector(tx, ty, 1)
    t = d / dir.z
    return t * dir

# Ebenen erzeugen
for d, col in distances:
    make_plane(d, col)

# Strahlen nur bis zur größten Distanz
dmax = max([d for d, _ in distances])

corner = ray_point(theta, theta, dmax)
edge   = ray_point(theta, 0, dmax)
mid    = ray_point(0, 0, dmax)

curve(pos=[vector(0,0,0), corner], color=color.black)
curve(pos=[vector(0,0,0), edge], color=color.black)
curve(pos=[vector(0,0,0), mid], color=color.black)

# --- Berechnungen für jedes d ---
print("\n=== FoV Werte ===")
for d, col in distances:
    # Strahlenlängen
    mid_d    = ray_point(0,0,d)
    edge_d   = ray_point(theta,0,d)
    corner_d = ray_point(theta,theta,d)

    # Quadratseite und Fläche
    side = 2 * d * tan(theta)
    area = side * side

    print(f"\nd = {d} m")
    print(f"  mid length    = {mag(mid_d):.4f}")
    print(f"  edge length   = {mag(edge_d):.4f}")
    print(f"  corner length = {mag(corner_d):.4f}")
    print(f"  FoV side      = {side:.4f}")
    print(f"  FoV area      = {area:.4f}")
