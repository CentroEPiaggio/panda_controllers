import casadi as ca

# Utility ---------------------------------------------------------------------
def clamp(x, low, high):
    return ca.fmax(low, ca.fmin(high, x))

def clamp01(x):
    return clamp(x, 0, 1)

# Closest points between two segments -----------------------------------------
def closestSegmentSegment(P0, P1, Q0, Q1, eps=1e-12):
    """
    Versione corretta che segue la logica di Ericson.
    Gestisce i casi di clamp e ricalcolo come in C++.
    """
    u = P1 - P0
    v = Q1 - Q0
    w = Q0 - P0

    a = ca.dot(u, u)
    b = ca.dot(u, v)
    c = ca.dot(v, v)
    d = ca.dot(u, w)
    e = ca.dot(v, w)
    D = a * c - b * b

    parallel = ca.fabs(D) < eps

    # Caso parallelo
    sN_par = 0.0
    sD_par = 1.0
    tN_par = e
    tD_par = c

    # Caso non parallelo
    sN_np = c * d - b * e
    tN_np = a * e - b * d
    sD_np = D
    tD_np = D

    # Selezione branch
    sN = ca.if_else(parallel, sN_par, sN_np)
    sD = ca.if_else(parallel, sD_par, sD_np)
    tN = ca.if_else(parallel, tN_par, tN_np)
    tD = ca.if_else(parallel, tD_par, tD_np)

    # Clamp di sN con ricalcolo di tN (come in C++)
    # Se sN < 0, setta sN=0 e ricalcola tN
    cond_sN_lt_0 = sN < 0.0
    tN_when_sN_lt_0 = e  # quando s=0
    
    # Se sN > sD, setta sN=sD e ricalcola tN
    cond_sN_gt_sD = sN > sD
    tN_when_sN_gt_sD = e + b  # quando s=1, w = Q0 - P1
    
    # Aggiorna tN in base al clamp di sN
    tN_updated = ca.if_else(cond_sN_lt_0, tN_when_sN_lt_0,
                   ca.if_else(cond_sN_gt_sD, tN_when_sN_gt_sD, tN))
    
    # Clamp sN
    sN_clamped = clamp(sN, 0.0, sD)
    
    # Ora clamp tN_updated
    cond_tN_lt_0 = tN_updated < 0.0
    cond_tN_gt_tD = tN_updated > tD
    
    # Clamp finale
    tN_clamped = clamp(tN_updated, 0.0, tD)
    
    # Calcola parametri finali
    sc = ca.if_else(ca.fabs(sD) < eps, 0.0, sN_clamped / sD)
    tc = ca.if_else(ca.fabs(tD) < eps, 0.0, tN_clamped / tD)

    Pc = P0 + sc * u
    Qc = Q0 + tc * v

    return sc, tc, Pc, Qc

# Capsule–Plane ( 3 casi come in C++) ---------------------------------
def dist_capsule_plane(P0, P1, r, plane_point, plane_normal):
    """
    Calcolo robusto della distanza capsula-piano.
    Gestisce i 3 casi: parallelo, intersezione, laterale.
    """
    # Normalizzazione robusta del vettore normale
    n_norm = ca.norm_2(plane_normal)
    n = plane_normal / (n_norm + 1e-12)
    
    # Distanze signed degli endpoint dal piano
    d0 = ca.dot(P0 - plane_point, n)
    d1 = ca.dot(P1 - plane_point, n)
    
    # Vettore del segmento
    u = P1 - P0
    u_dot_n = ca.dot(u, n)
    
    # Condizioni per i 3 casi
    is_parallel = ca.fabs(u_dot_n) < 1e-12
    is_intersecting = d0 * d1 <= 0.0
    
    # --- Caso 1: Segmento parallelo al piano ---
    t_parallel = 0.5
    P_parallel = P0 + t_parallel * u
    d_parallel = (d0 + d1) / 2.0
    
    # --- Caso 2: Capsula interseca il piano ---
    # Protezione: calcola denominatore sicuro solo se necessario
    denom_safe = ca.if_else(is_intersecting, d0 - d1, 1.0)
    t_intersect_raw = d0 / (denom_safe + 1e-12)
    t_intersect = clamp01(t_intersect_raw)
    P_intersect = P0 + t_intersect * u
    # Quando interseca, la distanza del segmento dal piano è 0
    
    # --- Caso 3: Capsula interamente da un lato ---
    use_A = ca.fabs(d0) <= ca.fabs(d1)
    t_side = ca.if_else(use_A, 0.0, 1.0)
    P_side = ca.if_else(use_A, P0, P1)
    d_side = ca.if_else(use_A, d0, d1)
    
    # --- Selezione finale con if_else annidati ---
    final_t = ca.if_else(
        is_parallel, 
        t_parallel,
        ca.if_else(is_intersecting, t_intersect, t_side)
    )
    
    final_P = ca.if_else(
        is_parallel, 
        P_parallel,
        ca.if_else(is_intersecting, P_intersect, P_side)
    )
    
    final_dist_signed = ca.if_else(
        is_parallel, 
        d_parallel,
        ca.if_else(is_intersecting, 0.0, d_side)
    )
    
    # Distanza finale = distanza segmento-piano - raggio capsula
    return ca.fabs(final_dist_signed) - r, final_P

# Capsule–Rectangle ------------------------------------------------
def dist_capsule_rectangle(P0, P1, r, center, axes, half_sizes):
    """
    Rettangolo definito in 3D con normale axes[2].
    half_sizes: [half_width, half_height, 0] o [half_width, half_height]
    """
    # Assicurati che half_sizes sia 3D
    if len(half_sizes) == 2:
        hx, hy = half_sizes[0], half_sizes[1]
        hz = 0.0
    else:
        hx, hy, hz = half_sizes[0], half_sizes[1], half_sizes[2]
    
    R = ca.hcat([axes[0], axes[1], axes[2]])
    P0_loc = ca.mtimes(R.T, P0 - center)
    P1_loc = ca.mtimes(R.T, P1 - center)
    
    # Funzione per proiettare su faccia rettangolo (Z=0 nel frame locale)
    def project_to_face(pt_loc):
        # Clamp su X e Y, Z=0
        return ca.vertcat(
            clamp(pt_loc[0], -hx, hx),
            clamp(pt_loc[1], -hy, hy),
            0.0
        )
    
    # Punti di vertice del rettangolo (nel frame locale)
    # Z=0 per tutti i vertici
    v0 = ca.vertcat(-hx, -hy, 0.0)
    v1 = ca.vertcat( hx, -hy, 0.0)
    v2 = ca.vertcat( hx,  hy, 0.0)
    v3 = ca.vertcat(-hx,  hy, 0.0)
    
    rect_edges = [(v0, v1), (v1, v2), (v2, v3), (v3, v0)]
    
    # Inizializza con il primo endpoint
    best_P_loc = P0_loc
    best_Q_loc = project_to_face(P0_loc)
    min_d2 = ca.sumsqr(P0_loc - best_Q_loc)
    
    # Test endpoint B
    Q_B = project_to_face(P1_loc)
    d2_B = ca.sumsqr(P1_loc - Q_B)
    
    cond_B = d2_B < min_d2
    min_d2 = ca.if_else(cond_B, d2_B, min_d2)
    best_P_loc = ca.if_else(cond_B, P1_loc, best_P_loc)
    best_Q_loc = ca.if_else(cond_B, Q_B, best_Q_loc)
    
    # Test 4 bordi
    for i, (E_start, E_end) in enumerate(rect_edges):
        s, t, p_seg, p_edge = closestSegmentSegment(P0_loc, P1_loc, E_start, E_end)
        d2_edge = ca.sumsqr(p_seg - p_edge)
        
        cond = d2_edge < min_d2
        min_d2 = ca.if_else(cond, d2_edge, min_d2)
        best_P_loc = ca.if_else(cond, p_seg, best_P_loc)
        best_Q_loc = ca.if_else(cond, p_edge, best_Q_loc)
    
    # Converti in coordinate mondo
    P_world = center + ca.mtimes(R, best_P_loc)
    Q_world = center + ca.mtimes(R, best_Q_loc)
    
    dist = ca.sqrt(min_d2)
    return dist - r, P_world, Q_world

# Capsule–Capsule --------------------------------------------------------------
def dist_capsule_capsule(A0, A1, rA, B0, B1, rB):
    s, t, PA, PB = closestSegmentSegment(A0, A1, B0, B1)
    diff = PA - PB
    d = ca.norm_2(diff)
    return d - (rA + rB), PA, PB, s, t

# Dispatcher -------------------------------------------------------------------
def capsule_distance(shape_type, **kwargs):
    if shape_type == "capsule":
        d, PA, PB, s, t = dist_capsule_capsule(
            kwargs['A0'], kwargs['A1'], kwargs['rA'],
            kwargs['B0'], kwargs['B1'], kwargs['rB']
        )
        return d, {'p_capsule': PA, 'p_obstacle': PB, 't_capsule': s, 't_obstacle': t}
    
    elif shape_type == "plane":
        d, P_closest = dist_capsule_plane(
            kwargs['P0'], kwargs['P1'], kwargs['r'],
            kwargs['plane_point'], kwargs['plane_normal']
        )
        return d, {'p_capsule': P_closest}
    
    elif shape_type == "rectangle":
        d, P_world, Q_world = dist_capsule_rectangle(
            kwargs['P0'], kwargs['P1'], kwargs['r'],
            kwargs['center'], kwargs['axes'], kwargs['half_sizes']
        )
        return d, {'p_capsule': P_world, 'p_obstacle': Q_world}
    
    else:
        raise ValueError(f"Unknown shape type: {shape_type}")

# Funzione helper per test
def test_capsule_distance():
    """Test rapido delle funzioni"""
    # Definisci variabili simboliche
    P0 = ca.SX.sym('P0', 3)
    P1 = ca.SX.sym('P1', 3)
    r = ca.SX.sym('r', 1)
    
    # Test capsule-capsula
    A0, A1 = ca.SX.sym('A0', 3), ca.SX.sym('A1', 3)
    B0, B1 = ca.SX.sym('B0', 3), ca.SX.sym('B1', 3)
    rA, rB = ca.SX.sym('rA', 1), ca.SX.sym('rB', 1)
    
    # Crea funzioni
    d_capsule, _ = dist_capsule_capsule(A0, A1, rA, B0, B1, rB)
    d_plane, _ = dist_capsule_plane(P0, P1, r, ca.DM([0,0,0]), ca.DM([0,0,1]))
    
    print("Funzioni create con successo")
    print("dist_capsule_capsule:", d_capsule)
    print("dist_capsule_plane:", d_plane)

if __name__ == "__main__":
    test_capsule_distance()