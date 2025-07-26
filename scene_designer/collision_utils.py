import numpy as np

# === 3D Collision Checks ===

def line_intersects_aabb(p1, p2, min_corner, max_corner):

    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    min_corner = np.array(min_corner, dtype=float)
    max_corner = np.array(max_corner, dtype=float)

    direction = p2 - p1
    tmin, tmax = 0.0, 1.0

    for i in range(3):  # Check x, y, z axes
        if abs(direction[i]) < 1e-8:  # Line is parallel to slab
            if p1[i] < min_corner[i] or p1[i] > max_corner[i]:
                return False  # Outside slab, no hit
        else:
            inv_dir = 1.0 / direction[i]
            t1 = (min_corner[i] - p1[i]) * inv_dir
            t2 = (max_corner[i] - p1[i]) * inv_dir
            t1, t2 = min(t1, t2), max(t1, t2)
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return False  # No overlap
    return True


def line_intersects_sphere(p1, p2, center, radius):

    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    center = np.array(center, dtype=float)

    d = p2 - p1        # Direction vector of the segment
    f = p1 - center    # Vector from center to p1

    a = np.dot(d, d)
    b = 2.0 * np.dot(f, d)
    c = np.dot(f, f) - radius ** 2

    discriminant = b ** 2 - 4 * a * c
    if discriminant < 0:
        return False  # No real roots => no intersection

    sqrt_disc = np.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2 * a)
    t2 = (-b + sqrt_disc) / (2 * a)

    return (0.0 <= t1 <= 1.0) or (0.0 <= t2 <= 1.0)


def line_intersects_cylinder(p1, p2, base, height, radius):

    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    base = np.array(base, dtype=float)
    axis = np.array(np.array([1, 0, 0]), dtype=float)

    axis = axis / np.linalg.norm(axis)  # Normalize axis
    d = p2 - p1                         # Segment direction
    m = p1 - base                       # Vector from base to p1

    n = d - np.dot(d, axis) * axis     # Remove axial component from segment direction
    o = m - np.dot(m, axis) * axis     # Remove axial component from base-to-p1

    a = np.dot(n, n)
    b = 2.0 * np.dot(n, o)
    c = np.dot(o, o) - radius ** 2

    discriminant = b ** 2 - 4 * a * c
    if discriminant < 0 or a == 0:
        return False  # No real roots or degenerate segment

    sqrt_disc = np.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2 * a)
    t2 = (-b + sqrt_disc) / (2 * a)

    for t in (t1, t2):
        if 0.0 <= t <= 1.0:
            pt = p1 + t * d
            proj_len = np.dot(pt - base, axis)
            if 0.0 <= proj_len <= height:
                return True

    return False

if(__name__ == "__main__"):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider
    from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

    aabb_min = np.array([0, 0, 0])
    aabb_max = np.array([5, 2, 5])

    sphere_center = np.array([4, 4, 4])
    sphere_radius = 1

    cyl_base = np.array([6, 3, 2])
    cyl_axis = np.array([1, 0, 0])
    cyl_height = 4
    cyl_radius = 1.0

    # -- Initial Line Segment --
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([7.0, 7.0, 7.0])


    # -- Visualization Setup --
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.2, bottom=0.4)

    line_seg = [p1, p2]
    line_collection = Line3DCollection([line_seg], colors='black', linewidths=2)
    ax.add_collection3d(line_collection)

    def draw_aabb(ax, min_corner, max_corner, color='green', alpha=0.3):
        # Create cube from min/max
        r = [min_corner, max_corner]
        verts = [
            [r[0], [r[0][0], r[0][1], r[1][2]], [r[0][0], r[1][1], r[1][2]], [r[0][0], r[1][1], r[0][2]]],
            [[r[1][0], r[0][1], r[0][2]], r[1], [r[1][0], r[1][1], r[0][2]], [r[1][0], r[0][1], r[1][2]]],
            [[r[0][0], r[0][1], r[1][2]], [r[1][0], r[0][1], r[1][2]], r[1], [r[0][0], r[1][1], r[1][2]]],
            [[r[0][0], r[1][1], r[0][2]], [r[1][0], r[1][1], r[0][2]], [r[1][0], r[1][1], r[1][2]], [r[0][0], r[1][1], r[1][2]]],
            [[r[0][0], r[0][1], r[0][2]], [r[1][0], r[0][1], r[0][2]], [r[1][0], r[0][1], r[1][2]], [r[0][0], r[0][1], r[1][2]]],
            [[r[0][0], r[0][1], r[0][2]], [r[0][0], r[1][1], r[0][2]], [r[1][0], r[1][1], r[0][2]], [r[1][0], r[0][1], r[0][2]]]
        ]
        box = Poly3DCollection(verts, alpha=alpha, facecolors=color)
        ax.add_collection3d(box)

    def draw_sphere(ax, center, radius, color='blue', alpha=0.2):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = center[0] + radius * np.cos(u) * np.sin(v)
        y = center[1] + radius * np.sin(u) * np.sin(v)
        z = center[2] + radius * np.cos(v)
        ax.plot_surface(x, y, z, color=color, alpha=alpha)

    def draw_cylinder(ax, base, radius, height, color='red', alpha=0.2):
        axis = np.array([1, 0, 0])
        axis = axis / np.linalg.norm(axis)
        t = np.linspace(0, height, 10)
        theta = np.linspace(0, 2*np.pi, 30)
        t, theta = np.meshgrid(t, theta)
        X = radius * np.cos(theta)
        Y = radius * np.sin(theta)
        Z = t
        # Create rotation matrix to align z with axis
        from scipy.spatial.transform import Rotation as R
        z_axis = np.array([0, 0, 1])
        v = np.cross(z_axis, axis)
        if np.linalg.norm(v) < 1e-6:
            rot = np.eye(3)
        else:
            c = np.dot(z_axis, axis)
            s = np.linalg.norm(v)
            vx = np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
            rot = np.eye(3) + vx + (vx @ vx) * ((1 - c) / (s**2))
        coords = np.dot(rot, np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=0))
        Xr = coords[0, :].reshape(X.shape) + base[0]
        Yr = coords[1, :].reshape(Y.shape) + base[1]
        Zr = coords[2, :].reshape(Z.shape) + base[2]
        ax.plot_surface(Xr, Yr, Zr, color=color, alpha=alpha)


    def update_line(val=None):
        p1 = np.array([sx1.val, sy1.val, sz1.val])
        p2 = np.array([sx2.val, sy2.val, sz2.val])
        line_collection.set_segments([[p1, p2]])

        # Recolor line based on intersection type
        hit = (
            line_intersects_aabb(p1, p2, aabb_min, aabb_max) or
            line_intersects_sphere(p1, p2, sphere_center, sphere_radius) or
            line_intersects_cylinder(p1, p2, cyl_base, cyl_axis, cyl_height, cyl_radius)
        )
        line_collection.set_color('orange' if hit else 'black')
        fig.canvas.draw_idle()

    # Draw the scene
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 8)
    ax.set_zlim(0, 8)
    ax.set_box_aspect([1,1,1])

    draw_aabb(ax, aabb_min, aabb_max, color='green', alpha=0.2)
    draw_sphere(ax, sphere_center, sphere_radius, color='blue', alpha=0.2)
    draw_cylinder(ax, cyl_base, cyl_axis, cyl_radius, cyl_height, color='red', alpha=0.2)

    # Sliders for p1 and p2
    slider_color = 'lightblue'
    slider_ax = lambda i: plt.axes([0.2, 0.35 - i*0.05, 0.65, 0.02], facecolor=slider_color)

    sx1 = Slider(slider_ax(0), 'p1.x', 0.0, 8.0, valinit=p1[0])
    sy1 = Slider(slider_ax(1), 'p1.y', 0.0, 8.0, valinit=p1[1])
    sz1 = Slider(slider_ax(2), 'p1.z', 0.0, 8.0, valinit=p1[2])

    sx2 = Slider(slider_ax(3), 'p2.x', 0.0, 8.0, valinit=p2[0])
    sy2 = Slider(slider_ax(4), 'p2.y', 0.0, 8.0, valinit=p2[1])
    sz2 = Slider(slider_ax(5), 'p2.z', 0.0, 8.0, valinit=p2[2])

    for s in [sx1, sy1, sz1, sx2, sy2, sz2]:
        s.on_changed(update_line)

    update_line()
    plt.show()