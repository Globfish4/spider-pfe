import torch
from scipy.spatial.transform import Rotation
import numpy as np



def normalize_angle(x):
    return torch.atan2(torch.sin(x), torch.cos(x))



@torch.jit.script
def copysign(a, b):
    # type: (float, Tensor) -> Tensor
    a = torch.tensor(a, device=b.device, dtype=torch.float).repeat(b.shape[0])
    return torch.abs(a) * torch.sign(b)


@torch.jit.script
def normalize(x, eps: float = 1e-9):
    return x / x.norm(p=2, dim=-1).clamp(min=eps, max=None).unsqueeze(-1)



@torch.jit.script
def quat_rotate(q, v):
    shape = q.shape
    q_w = q[:, 0]
    q_vec = q[:, 1:]
    a = v * (2.0 * q_w**2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a + b + c



@torch.jit.script
def quat_conjugate(a):
    shape = a.shape
    a = a.reshape(-1, 4)
    return torch.cat((a[:, 0:1], -a[:, 1:]), dim=-1).view(shape)


@torch.jit.script
def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, 0]
    q_vec = q[:, 1:]
    a = v * (2.0 * q_w**2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c



@torch.jit.script
def get_basis_vector(q, v):
    return quat_rotate(q, v)



@torch.jit.script
def quat_mul(a, b):
    assert a.shape == b.shape
    shape = a.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 4)

    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    quat = torch.stack([w, x, y, z], dim=-1).view(shape)

    return quat





@torch.jit.script
def quats_to_rot_matrices(quats):
    squeeze_flag = False
    if quats.dim() == 1:
        squeeze_flag = True
        quats = torch.unsqueeze(quats, 0)
    nq = torch.linalg.vecdot(quats, quats, dim=1)
    singularities = nq < 1e-10
    result = torch.zeros(quats.shape[0], 3, 3, device=quats.device)
    result[singularities] = torch.eye(3, device=quats.device).reshape((1, 3, 3)).repeat(sum(singularities), 1, 1)
    non_singular = quats[torch.logical_not(singularities)] * torch.sqrt(2.0 / nq).reshape((-1, 1)).repeat(1, 4)
    non_singular = torch.einsum("bi,bj->bij", non_singular, non_singular)
    result[torch.logical_not(singularities), 0, 0] = 1.0 - non_singular[:, 2, 2] - non_singular[:, 3, 3]
    result[torch.logical_not(singularities), 0, 1] = non_singular[:, 1, 2] - non_singular[:, 3, 0]
    result[torch.logical_not(singularities), 0, 2] = non_singular[:, 1, 3] + non_singular[:, 2, 0]
    result[torch.logical_not(singularities), 1, 0] = non_singular[:, 1, 2] + non_singular[:, 3, 0]
    result[torch.logical_not(singularities), 1, 1] = 1.0 - non_singular[:, 1, 1] - non_singular[:, 3, 3]
    result[torch.logical_not(singularities), 1, 2] = non_singular[:, 2, 3] - non_singular[:, 1, 0]
    result[torch.logical_not(singularities), 2, 0] = non_singular[:, 1, 3] - non_singular[:, 2, 0]
    result[torch.logical_not(singularities), 2, 1] = non_singular[:, 2, 3] + non_singular[:, 1, 0]
    result[torch.logical_not(singularities), 2, 2] = 1.0 - non_singular[:, 1, 1] - non_singular[:, 2, 2]
    if squeeze_flag:
        result = torch.squeeze(result)
    return result



@torch.jit.script
def matrices_to_euler_angles(mat, extrinsic: bool = True):
    _POLE_LIMIT = 1.0 - 1e-6
    if extrinsic:
        north_pole = mat[:, 2, 0] > _POLE_LIMIT
        south_pole = mat[:, 2, 0] < -_POLE_LIMIT
        result = torch.zeros(mat.shape[0], 3, device=mat.device)
        result[north_pole, 0] = 0.0
        result[north_pole, 1] = -np.pi / 2
        result[north_pole, 2] = torch.arctan2(mat[north_pole, 0, 1], mat[north_pole, 0, 2])
        result[south_pole, 0] = 0.0
        result[south_pole, 1] = np.pi / 2
        result[south_pole, 2] = torch.arctan2(mat[south_pole, 0, 1], mat[south_pole, 0, 2])
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0] = torch.arctan2(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2, 1],
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2, 2],
        )
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 1] = -torch.arcsin(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2, 0]
        )
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2] = torch.arctan2(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 1, 0],
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0, 0],
        )
    else:
        north_pole = mat[:, 2, 0] > _POLE_LIMIT
        south_pole = mat[:, 2, 0] < -_POLE_LIMIT
        result = torch.zeros(mat.shape[0], 3, device=mat.device)
        result[north_pole, 0] = torch.arctan2(mat[north_pole, 1, 0], mat[north_pole, 1, 1])
        result[north_pole, 1] = np.pi / 2
        result[north_pole, 2] = 0.0
        result[south_pole, 0] = torch.arctan2(mat[south_pole, 1, 0], mat[south_pole, 1, 1])
        result[south_pole, 1] = -np.pi / 2
        result[south_pole, 2] = 0.0
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0] = -torch.arctan2(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 1, 2],
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2, 2],
        )
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 1] = torch.arcsin(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0, 2]
        )
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2] = -torch.arctan2(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0, 1],
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0, 0],
        )
    return result






@torch.jit.script
def get_euler_xyz(q, extrinsic: bool = True):
    if extrinsic:
        qw, qx, qy, qz = 0, 1, 2, 3
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (q[:, qw] * q[:, qx] + q[:, qy] * q[:, qz])
        cosr_cosp = q[:, qw] * q[:, qw] - q[:, qx] * q[:, qx] - q[:, qy] * q[:, qy] + q[:, qz] * q[:, qz]
        roll = torch.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (q[:, qw] * q[:, qy] - q[:, qz] * q[:, qx])
        pitch = torch.where(torch.abs(sinp) >= 1, copysign(np.pi / 2.0, sinp), torch.asin(sinp))

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q[:, qw] * q[:, qz] + q[:, qx] * q[:, qy])
        cosy_cosp = q[:, qw] * q[:, qw] + q[:, qx] * q[:, qx] - q[:, qy] * q[:, qy] - q[:, qz] * q[:, qz]
        yaw = torch.atan2(siny_cosp, cosy_cosp)

        return roll % (2 * np.pi), pitch % (2 * np.pi), yaw % (2 * np.pi)
    else:
        result = matrices_to_euler_angles(quats_to_rot_matrices(q), extrinsic=False)
        return result[:, 0], result[:, 1], result[:, 2]








def estimate_velocity_from_positions(torso_positions, dt: float=1/60):
    """
    Approxime la vitesse par dérivée de la position.
    torso_positions: Tensor shape (N, 3)
    dt: intervalle de temps entre deux positions (constante)
    """
    # Différence avant/arrière
    vel = torch.zeros_like(torso_positions)
    vel[1:] = (torso_positions[1:] - torso_positions[:-1]) / dt
    vel[0] = vel[1]  # copie pour la première frame
    return vel











@torch.jit.script
def compute_heading_and_up(torso_rotation, inv_start_rot, to_target, vec0, vec1, up_idx):
    # type: (Tensor, Tensor, Tensor, Tensor, Tensor, int) -> Tuple[Tensor, Tensor, Tensor, Tensor, Tensor]
    num_envs = torso_rotation.shape[0]
    target_dirs = normalize(to_target)

    torso_quat = quat_mul(torso_rotation, inv_start_rot)
    up_vec = get_basis_vector(torso_quat, vec1).view(num_envs, 3)
    heading_vec = get_basis_vector(torso_quat, vec0).view(num_envs, 3)
    up_proj = up_vec[:, up_idx]
    heading_proj = torch.bmm(heading_vec.view(num_envs, 1, 3), target_dirs.view(num_envs, 3, 1)).view(num_envs)

    return torso_quat, up_proj, heading_proj, up_vec, heading_vec



@torch.jit.script
def compute_rot(torso_quat, velocity, ang_velocity, targets, torso_positions, extrinsic: bool = True):
    vel_loc = quat_rotate_inverse(torso_quat, velocity)
    angvel_loc = quat_rotate_inverse(torso_quat, ang_velocity)

    roll, pitch, yaw = get_euler_xyz(torso_quat, extrinsic=extrinsic)

    walk_target_angle = torch.atan2(targets[:, 2] - torso_positions[:, 2], targets[:, 0] - torso_positions[:, 0])
    angle_to_target = walk_target_angle - yaw

    return vel_loc, angvel_loc, roll, pitch, yaw, angle_to_target


def from_euler_to_quat(x, y, z):
    rot = Rotation.from_euler('xyz', [x, y, z], degrees=True)
    rot_quat = rot.as_quat()
    rot_quat = torch.tensor([[rot_quat[3], rot_quat[0], rot_quat[1], rot_quat[2]]], dtype=torch.float32, device="cuda:0")
    
    # print(rot_quat)
    return rot_quat


if __name__ == "__main__":

    torso_position = torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float32, device="cuda:0")
    velocity = torso_position # estimate_velocity_from_positions(torso_position)
    torso_quat = from_euler_to_quat(0.0, 0.0, 0.0)
    ang_velocity = torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float32, device="cuda:0")
    targets = torch.tensor([[1000, 0, 0]], dtype=torch.float32, device="cuda:0")

    vel_loc, angvel_loc, roll, pitch, yaw, angle_to_target = compute_rot(
    torso_quat, velocity, ang_velocity, targets, torso_position
    )

    print(vel_loc, angvel_loc, roll, pitch, yaw, angle_to_target)


    torso_rotation = torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float32, device="cuda:0")
    inv_start_rot = torch.tensor([[0.0]], dtype=torch.float32, device="cuda:0")
    to_target = targets - torso_position
    to_target[:, 2] = 0.0
    basis_vec0 = torch.tensor([[1, 0, 0]], dtype=torch.float32, device="cuda:0")
    basis_vec1 = torch.tensor([[0, 0, 1]], dtype=torch.float32, device="cuda:0")


    torso_quat, up_proj, heading_proj, up_vec, heading_vec = compute_heading_and_up(
        torso_rotation, inv_start_rot, to_target, basis_vec0, basis_vec1, 2
    )
    print(torso_quat, up_proj, heading_proj, up_vec, heading_vec)


