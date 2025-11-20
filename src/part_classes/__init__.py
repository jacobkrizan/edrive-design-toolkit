"""
Part Classes Module
Component classes for electric drive system
"""

from .motor import Motor
from .stator import Stator, StatorWindings, SlotLiner, StatorCore
from .rotor import Rotor, RotorCore, Magnet, Shaft, Endplate
from .inverter import Inverter
from .gearbox import Gearbox
from .battery import Battery

__all__ = [
    'Motor',
    'Stator', 'StatorWindings', 'SlotLiner', 'StatorCore',
    'Rotor', 'RotorCore', 'Magnet', 'Shaft', 'Endplate',
    'Inverter',
    'Gearbox',
    'Battery',
]
