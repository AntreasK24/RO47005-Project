import casadi as ca

def quat_mult(q1, q2):
    ans = ca.vertcat(
        q2[0, :] * q1[0, :]
        - q2[1, :] * q1[1, :]
        - q2[2, :] * q1[2, :]
        - q2[3, :] * q1[3, :],
        q2[0, :] * q1[1, :]
        + q2[1, :] * q1[0, :]
        - q2[2, :] * q1[3, :]
        + q2[3, :] * q1[2, :],
        q2[0, :] * q1[2, :]
        + q2[2, :] * q1[0, :]
        + q2[1, :] * q1[3, :]
        - q2[3, :] * q1[1, :],
        q2[0, :] * q1[3, :]
        - q2[1, :] * q1[2, :]
        + q2[2, :] * q1[1, :]
        + q2[3, :] * q1[0, :],
    )
    return ans


def quat_error(q, q_ref):
    q_aux = ca.vertcat(
        q[0, :] * q_ref[0, :]
        + q[1, :] * q_ref[1, :]
        + q[2, :] * q_ref[2, :]
        + q[3, :] * q_ref[3, :],
        -q[1, :] * q_ref[0, :]
        + q[0, :] * q_ref[1, :]
        + q[3, :] * q_ref[2, :]
        - q[2, :] * q_ref[3, :],
        -q[2, :] * q_ref[0, :]
        - q[3, :] * q_ref[1, :]
        + q[0, :] * q_ref[2, :]
        + q[1, :] * q_ref[3, :],
        -q[3, :] * q_ref[0, :]
        + q[2, :] * q_ref[1, :]
        - q[1, :] * q_ref[2, :]
        + q[0, :] * q_ref[3, :],
    )
    # attitude errors. SQRT have small quantities added (1e-3) to alleviate the derivative
    # not being defined at zero, and also because it's in the denominator
    q_att_denom = ca.sqrt(q_aux[0] * q_aux[0] + q_aux[3] * q_aux[3] + 1e-3)
    q_att = (
        ca.vertcat(
            q_aux[0] * q_aux[1] - q_aux[2] * q_aux[3],
            q_aux[0] * q_aux[2] + q_aux[1] * q_aux[3],
            q_aux[3],
        )
        / q_att_denom
    )
    return q_att


def rotate_quat(q1, v1):
    ans = quat_mult(
        quat_mult(q1, ca.vertcat(0, v1)),
        ca.vertcat(q1[0, :], -q1[1, :], -q1[2, :], -q1[3, :]),
    )
    return ca.vertcat(ans[1, :], ans[2, :], ans[3, :])  # to covert to 3x1 vec