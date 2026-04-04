from dataclasses import dataclass
import math



@dataclass
class LatticeConfig:
    spacing: float
    angular_spacing: float = math.pi / 4
    terminal_radius: float = 25.0
    
