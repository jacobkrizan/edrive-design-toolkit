# E-Drive Design Toolkit

Professional Python application for permanent magnet electric motor design and analysis for electric vehicles.

## Features

- **Interactive UI**: Tkinter-based interface with real-time parameter adjustment
- **Comprehensive Motor Models**: Surface-mount permanent magnet motors with configurable poles and slots
- **Electromagnetic Analysis**: Geometry-based Magnetic Equivalent Circuit (MEC) solver
- **Professional Visualization**: 
  - 2D cross-section views with rotor, stator, windings, and magnets
  - Flux density distribution contours
  - Airgap flux density plots
  - Animated rotor position control
- **Performance Metrics**:
  - Torque and power calculations
  - Efficiency and losses (copper, iron)
  - Flux density analysis
  - Thermal estimates
  - Material utilization

## Installation

1. Clone the repository:
```bash
git clone https://github.com/jacobkrizan/edrive-design-toolkit.git
cd edrive-design-toolkit
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Run the GUI Application

```bash
python main.py
```

### Run Tests

Validate the electromagnetic calculations and geometry generation:

```bash
python test_motor.py
```

## Project Structure

```
edrive-design-toolkit/
├── main.py                 # Main GUI application
├── motor_parameters.py     # Motor parameter definitions and metrics
├── geometry.py            # Geometric calculations and mesh generation
├── magnetic_circuit.py    # MEC solver for electromagnetic analysis
├── visualization.py       # Visualization and plotting functions
├── test_motor.py         # Comprehensive test suite
├── requirements.txt      # Python dependencies
└── README.md            # This file
```

## Motor Parameters

### Configurable Parameters

- **Basic Configuration**: Number of poles, number of slots
- **Geometry**: Stator/rotor diameters, airgap, stack length, magnet dimensions
- **Electrical**: Rated current, speed, voltage, winding configuration
- **Materials**: Magnet properties (Br, Hc), steel permeability

### Default Configuration

- 8 poles, 48 slots (surface-mount PM motor)
- 200mm stator outer diameter
- 1mm airgap
- 100mm stack length
- 5mm magnet thickness
- NdFeB magnets (Br = 1.2T)

## Electromagnetic Analysis

The toolkit uses a **Magnetic Equivalent Circuit (MEC)** approach:

1. Discretizes motor geometry into magnetic network nodes
2. Calculates reluctances for airgap, magnets, teeth, and yoke
3. Solves system of equations for magnetic potentials
4. Computes flux distribution and performance metrics

## Visualization

The application provides three synchronized views:

1. **Motor Cross-Section**: Detailed 2D rendering showing stator, rotor, magnets, and three-phase windings
2. **Flux Density Distribution**: Contour plot of magnetic flux density throughout the motor
3. **Airgap Flux Density**: Radial flux density vs. angular position

## Performance Metrics

Calculated outputs include:

- **Electromagnetic**: Torque, power, back-EMF, flux linkage
- **Flux Densities**: Airgap (peak), tooth, yoke
- **Efficiency**: Copper loss, iron loss, efficiency percentage
- **Torque Quality**: Ripple percentage, cogging torque
- **Thermal**: Current density, winding resistance, temperature rise
- **Utilization**: Torque/power density (Nm/kg, kW/kg)

## Technical Details

### Magnetic Equivalent Circuit

The MEC solver creates a network with:
- 360 airgap nodes for high angular resolution
- Nodes for each magnet, tooth, and yoke segment
- Rotor core nodes
- Sparse matrix solver for computational efficiency

### Winding Configuration

- 3-phase distributed winding
- Configurable turns per coil and parallel paths
- Automatic phase assignment to slots
- Alternating coil polarity for proper field interaction

## Testing

The test suite (`test_motor.py`) validates:
- Parameter initialization
- Geometry generation
- MEC solver accuracy
- Performance calculations
- Multiple motor configurations
- Rotor position variations

## Future Enhancements

Potential additions:
- Finite Element Analysis (FEA) integration
- Interior permanent magnet (IPM) topologies
- Thermal analysis with CFD
- Mechanical stress analysis
- Multi-objective optimization
- Export to CAD formats

## License

MIT License

## Author

Jacob Krizan
Expert in electric motor design for electric vehicles

## Acknowledgments

Built with Python, NumPy, SciPy, and Matplotlib for professional motor design analysis.
