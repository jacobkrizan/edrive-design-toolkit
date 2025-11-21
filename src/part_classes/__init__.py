"""
Part Classes Module
Component classes for electric drive system
"""

from .motor import Motor
from .stator import Stator
from .stator_core import StatorCore
from .stator_windings import StatorWindings
from .slot_liner import SlotLiner
from .rotor import Rotor
from .rotor_core import RotorCore
from .magnet import Magnet
from .shaft import Shaft
from .endplate import Endplate
from .inverter import Inverter
from .gearbox import Gearbox
from .battery import Battery
from .drive_unit import DriveUnit
from .powertrain import Powertrain

__all__ = [
    'Motor',
    'Stator', 'StatorWindings', 'SlotLiner', 'StatorCore',
    'Rotor', 'RotorCore', 'Magnet', 'Shaft', 'Endplate',
    'Inverter',
    'Gearbox',
    'Battery',
    'DriveUnit',
    'Powertrain',
]
