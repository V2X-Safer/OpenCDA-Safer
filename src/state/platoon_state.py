from state.state import State

class PlatoonState(State):
    def __init__(self, opt):
        super().__init__(opt['world'])
        self.vehicles = []
        
        # Initialize platoon parameters from config
        self.max_capacity = opt.platoon_base.max_capacity
        self.inter_gap = opt.platoon_base.inter_gap
        self.open_gap = opt.platoon_base.open_gap
        self.warm_up_speed = opt.platoon_base.warm_up_speed
        self.change_leader_speed = opt.platoon_base.change_leader_speed
        self.leader_speeds_profile = opt.platoon_base.leader_speeds_profile
        self.stage_duration = opt.platoon_base.stage_duration
        
        # Additional platoon state attributes
        self.platoon_id = None
        self.leader_id = None
        self.member_ids = []
        self.formation_time = 0
        self.current_speed_stage = 0
        self.speed_stage_time = 0