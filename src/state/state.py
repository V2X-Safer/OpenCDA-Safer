from abc import abstractmethod
import carla

class State:
    def __init__(self, simulation_config):
        self.cav_world = carla.Client('localhost', simulation_config['client_port'])

    @property
    def score(self):
        ''' calcute oracle '''
        pass
    
    @abstractmethod
    def update(self):
        # update state
        pass
    
    @abstractmethod
    def mutate(self):
        # mutate state
        pass
    