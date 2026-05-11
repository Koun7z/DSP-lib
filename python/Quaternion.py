import numpy as np

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_normalize(q):
    return q / np.linalg.norm(q)

def rotate_vector(q, v):
    vq = np.array([0.0, *v])
    return quat_mul(quat_mul(q, vq), quat_conj(q))[1:]

def quat_from_omega(omega, dt):
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = omega / np.linalg.norm(omega)
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), *(axis * s)])


def main():
    q1 = [1.0, 2.0, 3.0, 4.0]
    q2 = [5.0, 6.0, 7.0, 8.0]

    qq = quat_mul(q1, q2)
    print("q1 * q2 =", qq)

if __name__ == "__main__":
    main()