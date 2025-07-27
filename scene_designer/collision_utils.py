import numpy as np

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
