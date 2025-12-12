import torch

class Dynamics:
    def __init__(self, dt=0.05, device="cuda:0") -> None:
        self._dt = dt
        self._device = device
        self._L0 = 0.421   # truck wheelbase
        self._M0 = -0.045   # hitch offset behind truck rear axle
        self._L1 =  0.495   # hitch to trailer axle

    
    def step(self, states: torch.Tensor, actions: torch.Tensor, t: int) -> torch.Tensor:
        """
        Compute state derivatives using kinematic model.
        
        state = [x1, y1, theta0, theta1]
        u = [V0, delta0]
        """
        x1, y1, th0, th1 = states[:, 0], states[:, 1], states[:, 2], states[:, 3]
        V0, delta0 = actions[:, 0], actions[:, 1]
        
        # Articulation angle
        beta = th0 - th1

        # wrap angle between -pi and pi and constraint between -pi/2 and pi/2
        # beta = (beta + torch.pi) % (2 * torch.pi) - torch.pi
        # beta = torch.clamp(beta, -torch.pi/2 + 0.01, torch.pi/2 - 0.01)
        
        # Trailer velocity (kinematic relation)
        V1 = V0 * torch.cos(beta) + self._M0 * (V0 * torch.tan(delta0) / self._L0) * torch.sin(beta)
        
        # State derivatives
        dx1 = V1 * torch.cos(th1)
        dy1 = V1 * torch.sin(th1)
        
        dth1 = (V0 * torch.sin(beta)) / self._L1 - \
               (self._M0 * (V0 * torch.tan(delta0) / self._L0) * torch.cos(beta)) / self._L1
        
        dth0 = V0 * torch.tan(delta0) / self._L0
        
        new_x1 = x1 + dx1 * self._dt
        new_y1 = y1 + dy1 * self._dt
        new_th1 = th1 + dth1 * self._dt
        new_th0 = th0 + dth0 * self._dt
        
        # new_states = states
        new_states = states.clone()

        new_states[:, 0] = new_x1
        new_states[:, 1] = new_y1
        new_states[:, 2] = new_th0
        new_states[:, 3] = new_th1

        return new_states, actions