# Jacobiano inverso (com verificação de singularidade)
def inverse_jacobian(q1, q2):
    sin_q2 = mt.sin(q2)
    if abs(sin_q2) < 1e-6:
        raise ValueError("Jacobian is singular (q2 ≈ 0), cannot invert.")

    denom = a1 * a2 * sin_q2
    return (1 / denom) * np.array(
        [
            [a2 * mt.cos(q1 + q2), a2 * mt.sin(q1 + q2)],
            [
                -a1 * mt.cos(q1) - a2 * mt.cos(q1 + q2),
                -a1 * mt.sin(q1) - a2 * mt.sin(q1 + q2),
            ],
        ]
    )
