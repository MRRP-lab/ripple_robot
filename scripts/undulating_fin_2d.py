import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================================
# D2Q9 LBM PARAMETERS (SIDE VIEW x–z)
# ============================================================

nx = 280            # grid points in x (streamwise direction)
nz = 120            # grid points in z (fin-width direction)
omega = 1.7         # relaxation parameter  (1 < omega < 2)
nu = (1.0 / omega - 0.5) / 3.0   # kinematic viscosity (for reference)

timesteps = 6000    # total LBM steps
plot_every = 5      # steps per animation frame

# ============================================================
# FIN GEOMETRY (LATTICE UNITS; THINK OF z AS "FIN WIDTH")
# ============================================================

# Thickness H_fin is the main parameter you will vary
H_fin = 18          # approximate fin thickness (grid cells)  <== change this

A_fin = 8           # wave amplitude in z (grid cells)
lambda_fin = nx / 1.5   # spatial wavelength along x (grid cells)
f_wave = 0.001      # temporal wave frequency (cycles per time-step)

z_root = 8          # "root" offset above bottom wall where fin attaches

# x-span where the fin exists (to get rounded nose/tail)
x_start = 30
x_end   = nx - 30   # exclusive upper bound

# ============================================================
# D2Q9 LATTICE SETUP
# ============================================================

# Velocity set c_i
c = np.array([
    [ 0,  0],  # 0: rest
    [ 1,  0],  # 1: east      ( +x )
    [ 0,  1],  # 2: north     ( +z )
    [-1,  0],  # 3: west      ( -x )
    [ 0, -1],  # 4: south     ( -z )
    [ 1,  1],  # 5: north-east
    [-1,  1],  # 6: north-west
    [-1, -1],  # 7: south-west
    [ 1, -1],  # 8: south-east
], dtype=np.int32)

# Weights w_i
w = np.array([
    4/9,      # 0
    1/9, 1/9, 1/9, 1/9,      # 1-4
    1/36, 1/36, 1/36, 1/36   # 5-8
], dtype=np.float64)

opposite = np.array([0, 3, 4, 1, 2, 7, 8, 5, 6], dtype=np.int32)

# ============================================================
# MACROSCOPIC FIELDS AND DISTRIBUTIONS
# ============================================================

rho = np.ones((nz, nx), dtype=np.float64)       # density
ux  = np.zeros((nz, nx), dtype=np.float64)      # x-velocity
uz  = np.zeros((nz, nx), dtype=np.float64)      # z-velocity

f = np.zeros((9, nz, nx), dtype=np.float64)     # populations f_i(z, x)

def equilibrium(rho, ux, uz):
    """
    Compute D2Q9 equilibrium distributions for given rho, ux, uz.
    Returns array feq with shape (9, nz, nx).
    """
    cu = np.zeros_like(f)
    usq = ux**2 + uz**2
    for i in range(9):
        ci = c[i]
        cu[i] = 3.0 * (ci[0] * ux + ci[1] * uz)
    feq = np.empty_like(f)
    for i in range(9):
        feq[i] = w[i] * rho * (1 + cu[i] + 0.5 * cu[i]**2 - 1.5 * usq)
    return feq

# Initialize distributions at rest
f[:] = equilibrium(rho, ux, uz)

# ============================================================
# STATIC WALLS (BOTTOM AND TOP)
# ============================================================

solid_static = np.zeros((nz, nx), dtype=bool)
solid_static[0, :]  = True   # bottom wall (z = 0)
solid_static[-1, :] = True   # top wall   (z = nz-1)

# Time-varying fin mask (True where fin solid is present)
fin_mask = np.zeros((nz, nx), dtype=bool)

# ============================================================
# FIN MASK GENERATION (SIDE VIEW, CONSTANT THICKNESS ALONG NORMAL)
# ============================================================

def update_fin_mask(t):
    """
    Update fin_mask(z, x) to represent a wavy fin at time t.

    Side view:
    - x-axis: 0 .. nx-1 (nose to tail)
    - z-axis: 0 .. nz-1 (body to fin tip)
    The fin is a ribbon whose centerline follows
        z_c(x, t) = z_root + H_fin/2 + A_fin * sin( 2π ( (x-x0)/λ - f_wave t ) )
    for x in [x_start, x_end).
    Thickness H_fin is applied approximately along the local normal
    so the fin looks more like a constant-thickness ribbon.
    """
    global fin_mask

    fin_mask[:, :] = False

    xs = np.arange(nx)
    # Only the active portion of xs is used
    xs_local = xs[x_start:x_end]

    # Centerline profile
    phase = 2.0 * np.pi * ((xs_local - x_start) / lambda_fin - f_wave * t)
    z_center = z_root + 0.5 * H_fin + A_fin * np.sin(phase)

    # For derivative, pad endpoints with one-sided differences
    dzdx = np.zeros_like(z_center)
    dzdx[1:-1] = 0.5 * (z_center[2:] - z_center[:-2])
    dzdx[0]    = z_center[1] - z_center[0]
    dzdx[-1]   = z_center[-1] - z_center[-2]

    # For each centerline point, lay down a "strip" along the local normal
    # This approximates constant thickness along the fin.
    # Number of sample points across thickness
    n_samples = int(max(6, 2 * H_fin))   # oversample to avoid holes
    s_vals = np.linspace(-0.5 * H_fin, 0.5 * H_fin, n_samples)

    for idx, x_c in enumerate(xs_local):
        z_c  = z_center[idx]
        dzdx_local = dzdx[idx]

        # tangent vector (dx, dz) ~ (1, dzdx)
        tx, tz = 1.0, dzdx_local
        norm_t = np.hypot(tx, tz)
        if norm_t == 0:
            # exactly vertical; choose normal pointing +z
            nx_hat, nz_hat = 0.0, 1.0
        else:
            tx /= norm_t
            tz /= norm_t
            # normal vector (rotate tangent by -90°): n = (tz, -tx)
            nx_hat, nz_hat = tz, -tx

        for s in s_vals:
            xk = x_c + nx_hat * s
            zk = z_c + nz_hat * s
            ix = int(round(xk))
            iz = int(round(zk))
            if 0 < ix < nx-1 and 1 <= iz < nz-1:   # avoid outer static walls
                fin_mask[iz, ix] = True

# ============================================================
# LBM STEP (COLLISION, STREAMING, BOUNCE-BACK)
# ============================================================

def collide_and_stream(f, rho, ux, uz, t):
    """
    Perform one BGK collision + streaming step with
    time-varying fin geometry.
    """
    global fin_mask

    # 1) Update fin geometry
    update_fin_mask(t)
    solid = solid_static | fin_mask

    # 2) Collision (BGK)
    feq = equilibrium(rho, ux, uz)
    f[:] = (1.0 - omega) * f + omega * feq

    # 3) Streaming (periodic in x, closed in z)
    #    We use np.roll for simplicity.
    for i in range(9):
        ci_x, ci_z = c[i]
        f[i] = np.roll(np.roll(f[i], ci_x, axis=1), ci_z, axis=0)

    # 4) Bounce-back at all solid nodes (simple stationary-wall reflection).
    #    This is not a moving-wall boundary yet; the fin forces the flow
    #    only via its changing geometry.
    for i in range(9):
        ip = opposite[i]
        fi  = f[i,  solid].copy()
        fip = f[ip, solid].copy()
        f[i,  solid] = fip
        f[ip, solid] = fi

    # 5) Recompute macroscopic fields
    rho[:, :] = np.sum(f, axis=0)
    ux[:, :] = 0.0
    uz[:, :] = 0.0
    for i in range(9):
        ux += c[i, 0] * f[i]
        uz += c[i, 1] * f[i]
    ux /= rho
    uz /= rho

    # Enforce zero velocity in solid cells
    ux[solid] = 0.0
    uz[solid] = 0.0

    return f, rho, ux, uz, solid

# ============================================================
# ANIMATION SETUP
# ============================================================

fig, ax = plt.subplots(figsize=(9, 4))

speed = np.sqrt(ux**2 + uz**2)
im = ax.imshow(speed,
               origin='lower',
               cmap='viridis',
               vmin=0.0, vmax=0.12,
               extent=[0, nx, 0, nz])

# Semi-transparent overlay showing fin location
fin_overlay = ax.imshow(fin_mask.astype(float),
                        origin='lower',
                        cmap='gray',
                        alpha=0.5,
                        vmin=0, vmax=1,
                        extent=[0, nx, 0, nz])

cb = fig.colorbar(im, ax=ax)
cb.set_label("Velocity magnitude (lattice units)")

ax.set_xlabel("x (streamwise)")
ax.set_ylabel("z (fin width)")
title = ax.set_title("D2Q9 LBM – Side View of Undulating Fin")

info_text = ax.text(0.01, 0.97, "",
                    transform=ax.transAxes,
                    ha="left", va="top",
                    fontsize=9, color="white")

def update(frame):
    global f, rho, ux, uz, fin_mask
    # Advance LBM several steps between frames
    for _ in range(plot_every):
        t = frame * plot_every + _
        f, rho, ux, uz, solid = collide_and_stream(f, rho, ux, uz, t)

    speed = np.sqrt(ux**2 + uz**2)
    im.set_data(speed)
    fin_overlay.set_data(fin_mask.astype(float))
    info_text.set_text(f"Time step: {frame * plot_every}")
    return im, fin_overlay, info_text

anim = FuncAnimation(fig,
                     update,
                     frames=timesteps // plot_every,
                     interval=30,
                     blit=False)

plt.tight_layout()
plt.show()