import torch

class Objective(object):
    def __init__(self, goal, pre_processed_map, device="cuda:0"):
        self.nav_goal = torch.tensor(goal, device=device).unsqueeze(0)
        self._goal_weight = 1.0
        self._back_vel_weight = 1.0
        self._map_data = pre_processed_map
    
    def compute_running_cost(self, state: torch.Tensor):
        goal_dist = self._goal_cost(state[:, 0:2])
        collision_cost = self.truck_trailer_collision(state)
        return goal_dist * self._goal_weight + collision_cost

    def _goal_cost(self, position):
        return torch.linalg.norm(position - self.nav_goal, axis=1)
    

    def _check_collision_torch(self, state, collision_cost=1e6):
        """
        trajs: (N, T, 2) tensor of world positions (x,y) in meters
        map_data: output of preprocess_occupancy_grid()

        Returns:
            collision_mask: (N,) bool tensor (True = collides)
        """

        device = state.device

        occ    = self._map_data['occ']           # (H, W)
        origin = self._map_data['origin']        # (2,)
        res    = self._map_data['resolution']    # (1,)
        yaw    = self._map_data['yaw']
        W      = self._map_data['W']
        H      = self._map_data['H']

        # shift points to map origin
        # dx = state[..., 0] - origin[0]
        # dy = state[..., 1] - origin[1]

        dx = state[:, 0] - origin[0]
        dy = state[:, 1] - origin[1]

        # rotate points into map frame
        cos_yaw = torch.cos(yaw)
        sin_yaw = torch.sin(yaw)
        x_map =  cos_yaw * dx + sin_yaw * dy
        y_map = -sin_yaw * dx + cos_yaw * dy

        # convert to grid indices
        gx = (x_map / res).long()
        gy = (y_map / res).long()

        # check bounds — out of map = collision
        out_of_bounds = (
            (gx < 0) | (gy < 0) | (gx >= W) | (gy >= H)
        )  # (N,T)

        # mask inside-map points
        in_bounds = ~out_of_bounds

        # extract occupancy at valid locations
        # flatten indices for gather
        flat_indices = gy[in_bounds] * W + gx[in_bounds]

        occ_flat = occ.view(-1)  # flatten (H*W)

        occupied = torch.zeros_like(out_of_bounds, dtype=torch.bool, device=device)
        occupied[in_bounds] = occ_flat[flat_indices] > 0

        # per-trajectory collision
        # collision if ANY timestep hits occupied or out-of-bounds
        # collision_mask = (occupied | out_of_bounds).any(dim=1)
        collision_mask = occupied | out_of_bounds

        # assign high cost to samples in collision or leaving map
        costs = torch.zeros(state.shape[0], device=state.device)
        costs[collision_mask] = collision_cost

        return costs  # (N,)
    

    import torch

    
    def truck_trailer_collision(
        self,
        states,         # (N,4) -> [x1,y1,th0,th1]
        L0= 0.42,  # truck wheelbase
        M0=-0.02,  # hitch offset behind truck rear axle
        L1= 0.537,   # hitch → trailer axle
        W0= 0.35,    # truck width
        W1= 0.40,    # trailer width
        collision_cost = 1e6
    ):
        """
        Batched collision check for truck+trailer using perimeter sampling.
        Returns boolean tensor: collision = True if any rectangle point collides or out-of-bounds
        """

        device = states.device
        N = states.shape[0]

        x1, y1, th0, th1 = states[:,0], states[:,1], states[:,2], states[:,3]   # (x1, y1) trailer real axle
        occ    = self._map_data['occ']           # (H, W)
        map_origin = self._map_data['origin']        # (2,)
        res    = self._map_data['resolution']    # (1,)
        map_yaw    = self._map_data['yaw']
        W      = self._map_data['W']
        H      = self._map_data['H']


        # --------------------------
        # 1) Compute truck rear axle
        # --------------------------
        # Hitch position behind trailer axle
        hitch_x = x1 + L1 * torch.cos(th1)
        hitch_y = y1 + L1 * torch.sin(th1)

        # Truck rear axle behind hitch
        truck_rear_x = hitch_x + M0 * torch.cos(th0)
        truck_rear_y = hitch_y + M0 * torch.sin(th0)

        # --------------------------
        # 2) Rectangle perimeter points
        # --------------------------
        K = 16  # points per edge

        def perimeter_points(L, W, Np=16):
            xs = torch.cat([
                torch.linspace(-L/2, L/2, Np),
                torch.full((Np,), L/2),
                torch.linspace(L/2, -L/2, Np),
                torch.full((Np,), -L/2)
            ])
            ys = torch.cat([
                torch.full((Np,), -W/2),
                torch.linspace(-W/2, W/2, Np),
                torch.full((Np,), W/2),
                torch.linspace(W/2, -W/2, Np)
            ])
            return torch.stack([xs, ys], dim=1).to(device)

        truck_local   = perimeter_points(L0, W0, K).unsqueeze(0).expand(N,-1,-1)   # (N, 4K, 2)
        trailer_local = perimeter_points(L1, W1, K).unsqueeze(0).expand(N,-1,-1)

        
        # --------------------------
        # 3) Rotate rectangles to world frame
        # --------------------------
        def rotate_translate(local_pts, yaw, cx, cy):
            # yaw: (N,)
            c = torch.cos(yaw)
            s = torch.sin(yaw)
            R = torch.stack([torch.stack([c,-s],dim=1), torch.stack([s,c],dim=1)], dim=1)  # (N,2,2)
            pts = (R @ local_pts.transpose(1,2)).transpose(1,2)  # (N,K,2)
            pts[...,0] += cx.unsqueeze(1)
            pts[...,1] += cy.unsqueeze(1)
            return pts

        truck_pts   = rotate_translate(truck_local, th0, truck_rear_x, truck_rear_y)
        trailer_pts = rotate_translate(trailer_local, th1, x1, y1)

        # --------------------------
        # 4) Transform points into MAP frame (account for origin + yaw)
        # --------------------------
        # Translate relative to map origin
        pts = torch.cat([truck_pts, trailer_pts], dim=1)  # (N, Ktot, 2)

        px = pts[...,0] - map_origin[0]
        py = pts[...,1] - map_origin[1]

        # Rotate into grid frame
        cos_yaw = torch.cos(map_yaw)
        sin_yaw = torch.sin(map_yaw)

        px_map =  cos_yaw * px + sin_yaw * py
        py_map = -sin_yaw * px + cos_yaw * py

        # Convert to grid indices
        gx = (px_map / res).long()
        gy = (py_map / res).long()

        # --------------------------
        # 5) Collision + out-of-bounds
        # --------------------------
        out_of_bounds = (gx < 0) | (gx >= W) | (gy < 0) | (gy >= H)

        # clamp indices to valid range for occupancy lookup
        gx_cl = gx.clamp(0, W-1)
        gy_cl = gy.clamp(0, H-1)

        occupied = occ[gy_cl, gx_cl]  # (N, Ktot) bool

        collision_mask = (occupied | out_of_bounds).any(dim=1).bool()  # (N,) bool

        # assign high cost to samples in collision or leaving map
        costs = torch.zeros(states.shape[0], device=states.device)
        costs[collision_mask] = collision_cost

        return costs

