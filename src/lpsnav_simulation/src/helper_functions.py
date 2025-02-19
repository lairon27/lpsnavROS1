import numpy as np

def rotate(obj, angle):
    if isinstance(angle, np.ndarray):
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return np.transpose(obj @ rot_matrix, axes=(1, 2, 0))
    rot_matrix = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
    return obj @ rot_matrix

def angle(vec):
    return np.arctan2(vec[1], vec[0]) if np.ndim(vec) == 1 else np.arctan2(vec[:, 1], vec[:, 0])

def dist(A, B):
    """
    Calculate Euclidean distance between two points A and B.
    A and B can be lists or arrays of coordinates.
    """
    A = np.array(A)
    B = np.array(B)
    
    try:
        difference = B - A
    except ValueError:
        difference = np.reshape(np.subtract.outer(B, A), (-1, 2))
    return np.sqrt(np.sum(np.square(difference), axis=-1))

def polar_ellipse(a, b, theta):
    return a * b / np.sqrt((b * np.cos(theta))**2 + (a * np.sin(theta))**2)

def path_len(path):
    return np.sum(np.linalg.norm(np.diff(path, axis=0), axis=-1))

def wrap_to_pi(th):
    return np.arctan2(np.sin(th), np.cos(th))

def vec(th):
    return np.squeeze(np.column_stack((np.cos(th), np.sin(th))))

def unit_vec(vec):
    return vec / np.linalg.norm(vec, axis=-1)

def in_front(pos, th, pt):
    return np.sum((pt - pos) * vec(th), axis=-1) > 0

def clip(vec, mag):
    scale = min(1, mag / np.linalg.norm(vec))
    return scale * vec

def directed_masks(pt, pt_vel, line, line_vel, t_to_line):
    t_to_line = np.clip(t_to_line, a_min=None, a_max=1e5)
    pred_pt = pt + t_to_line[..., None] * pt_vel
    pred_line = line[:, None, None, :] + t_to_line[..., None] * line_vel
    w = pred_pt - pred_line[0]
    v = pred_line[1] - pred_line[0]
    c0 = np.sum(w * v, axis=-1)
    c1 = np.sum(v * v, axis=-1)
    return np.array([c0 <= 0, (c0 > 0) & (c1 > c0), c1 <= c0])

def dynamic_pt_cost(pt, pt_speed, line, line_th, line_vel):
    cost_col_0 = cost_to_pt(pt, pt_speed, line[0], line_vel)
    cost_col_1 = cost_to_pt(pt, pt_speed, line[1], line_vel)
    cost_line = cost_to_line(pt, pt_speed, line, line_vel)
    masks = p_intersect(pt, pt_speed, line, line_th, line_vel, cost_line)
    masked_costs = masked_cost(masks, cost_col_0, cost_col_1, cost_line)
    return masked_costs

def dynamic_prim_cost(pos, pt, pt_speed, pt_vel, pred_line, line_th, line_vel, line):
    cost_col_0 = cost_to_pt(pt, pt_speed, pred_line[0], line_vel)
    cost_col_1 = cost_to_pt(pt, pt_speed, pred_line[1], line_vel)
    cost_line = cost_to_line(pt, pt_speed, pred_line, line_vel)
    masks = p_intersect(pt, pt_speed, pred_line, line_th, line_vel, cost_line)
    prim_costs = masked_cost(masks, cost_col_0, cost_col_1, cost_line)
    if np.any(~in_front(pred_line[0], line_th, pt)):
        dir_costs = directed_cost_to_line(pos, pt_vel, line, line_vel)
        dir_masks = directed_masks(pos, pt_vel, line, line_vel, dir_costs)
        masked = np.where(dir_masks, 0, prim_costs)
        return np.where(~in_front(pred_line[0], line_th, pt), masked, prim_costs)
    return prim_costs

def cost_to_pt(pos_0, speed_0, pos_1, vel_1=np.array([0, 0])):
    d_vec = pos_0 - pos_1
    d = np.linalg.norm(d_vec, axis=-1)
    d_vec_hat = d_vec / d[..., None]
    speed_1_t = np.abs(np.sum(rotate(d_vec_hat, np.pi / 2) * vel_1, axis=-1))
    speed_1_r = np.sum(d_vec_hat * vel_1, axis=-1)
    if not speed_0:
        speed_0_r = 0
    else:
        arg = 1 - (speed_1_t / speed_0) ** 2
        speed_0_r = speed_0 * np.sqrt(np.where(arg < 0, 0, arg))
    den = speed_0_r + speed_1_r
    with np.errstate(divide="ignore"):
        t = d / den
    t = np.where((t <= 0) | (speed_0 <= speed_1_t), np.inf, t)
    return np.nan_to_num(t)

def directed_cost_to_line(pos, pt_vel, line, line_vel):
    line_vec = line[1] - line[0]
    v0 = np.array([line_vec[1], -line_vec[0]])
    v1 = -v0
    r = pos - line[0]
    v = v0 if np.dot(v0, r) >= 0 else v1
    v_hat = v / np.linalg.norm(v)
    d = np.abs(np.dot(r, v_hat))
    proj_pt_speed = np.dot(pt_vel, -v_hat)
    proj_line_speed = np.dot(line_vel, v_hat)
    den = proj_pt_speed + proj_line_speed
    with np.errstate(divide="ignore"):
        t = d / den
    t = np.where(t <= 0, np.inf, t)
    return np.nan_to_num(t)

def dist_to_line_seg(pt, seg0, seg1):
    return np.linalg.norm(nearest_pt_on_line_seg(pt, seg0, seg1), axis=-1)

def nearest_pt_on_line_seg(pt, seg0, seg1):
    t = np.dot(pt - seg0, seg1 - seg0) / np.linalg.norm(seg1 - seg0)**2
    t = np.minimum(1, np.maximum(0, t))
    return seg0 + t[...,None] * (seg1 - seg0) - pt

def cost_to_line_th(p1, p1_speed, p2, p2_vel, th):
    dx, dy = np.subtract(p1, p2).T
    d = np.abs(np.cos(th) * dy - np.sin(th) * dx)
    proj_p2_speed = np.sum(vec(th) * p2_vel, axis=-1)
    with np.errstate(divide="ignore"):
        t = d / (p1_speed + proj_p2_speed)
    t = np.where(t <= 0, np.inf, t)
    return np.nan_to_num(t)

def cost_to_line(pt, pt_speed, line, line_vel):
    v0 = rotate(line[1] - line[0], np.pi / 2)
    v1 = -v0
    r = pt - line[0]
    v = np.where(np.sum(v0 * r, axis=-1)[..., None] > 0, v0, v1)
    v_hat = v / np.linalg.norm(v, axis=-1)[..., None]
    d = np.abs(np.sum(r * v_hat, axis=-1))
    proj_line_speed = np.sum(v_hat * line_vel, axis=-1)
    den = pt_speed + proj_line_speed
    with np.errstate(divide="ignore"):
        t = d / den
    t = np.where(t <= 0, np.inf, t)
    return np.nan_to_num(t)

def p_intersect(pos, v, line_pts, line_th, pt_vel, t_to_line):
    t_to_line = np.clip(t_to_line, a_min=None, a_max=1e5)
    vel = v * vec(wrap_to_pi(line_th + np.pi))
    r_pred = pos + vel * t_to_line[..., None]
    p0_pred = line_pts[0] + pt_vel * np.expand_dims(t_to_line, axis=-1)
    p1_pred = line_pts[1] + pt_vel * np.expand_dims(t_to_line, axis=-1)
    w = np.nan_to_num(r_pred - p0_pred)
    v = np.nan_to_num(p1_pred - p0_pred)
    c0 = np.sum(w * v, axis=-1)
    c1 = np.sum(v * v, axis=-1)
    return np.stack((c0 <= 0, (c0 > 0) & (c1 > c0), c1 <= c0))

def is_intersecting(s1, s2, l1, l2):
    p13 = s1 - l1
    p34 = l1 - l2
    p12 = s1 - s2
    num = p13[0] * p34[1] - p13[1] * p34[0]
    den = p12[0] * p34[1] - p12[1] * p34[0]
    return 0 <= num / den <= 1

def masked_cost(masks, cost_col_0, cost_col_1, cost_line):
    left = masks[0] * np.stack((cost_line, cost_col_0, cost_col_1))
    center = masks[1] * np.stack((cost_col_0, cost_line, cost_col_1))
    right = masks[2] * np.stack((cost_col_0, cost_col_1, cost_line))
    return left + center + right

