# E-Drive Design Toolkit

Professional Python application for permanent magnet electric motor design and analysis for electric vehicles.

## Features

- **Interactive UI**: Matplotlib-based interface with real-time parameter adjustment
- **Comprehensive Motor Models**: Surface-mount permanent magnet motors with configurable poles and slots
- **Dual Solver Architecture**: 
  - Fast Magnetic Equivalent Circuit (MEC) solver for rapid design iterations
  - High-fidelity Finite Element Analysis (FEA) using GetDP for detailed electromagnetic analysis
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

### Prerequisites

- **Python 3.8+**: Required for running the application
- **GetDP 3.x**: Professional electromagnetic solver (companion to gmsh)
  - Download from: https://getdp.info/
  - Extract to `getdp-3.5.0-Windows64/` in the project directory
  - Or install system-wide and update path in `fea_solver.py`

### Setup

1. Clone the repository:
```bash
git clone https://github.com/jacobkrizan/edrive-design-toolkit.git
cd edrive-design-toolkit
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. Install GetDP (Windows):
   - Download GetDP from https://getdp.info/
   - Extract to `getdp-3.5.0-Windows64/` in project folder
   - Executable should be at `getdp-3.5.0-Windows64/getdp.exe`

## Usage

### Run the GUI Application

```bash
python main.py
```

The application provides:
- **MEC Solver Tab**: Fast analytical calculations for rapid design exploration
- **FEA Solver Tab**: High-fidelity electromagnetic simulation using GetDP
- Real-time parameter updates and visualization

## Project Structure

```
edrive-design-toolkit/
├── main.py                      # Main GUI application
├── motor_parameters.py          # Motor parameter definitions and metrics
├── geometry.py                  # Geometric calculations and mesh generation
├── magnetic_circuit.py          # MEC solver for electromagnetic analysis
├── fea_solver.py                # FEA solver using GetDP
├── visualization.py             # Visualization and plotting functions
├── requirements.txt             # Python dependencies
├── README.md                    # This file
└── getdp-3.5.0-Windows64/       # GetDP electromagnetic solver
    ├── getdp.exe                # GetDP executable
    ├── examples/                # Example problems
    └── templates/               # GetDP formulation templates
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

The toolkit provides two complementary solvers:

### MEC Solver (Fast)

Magnetic Equivalent Circuit approach for rapid design iterations:

1. Discretizes motor geometry into magnetic network nodes
2. Calculates reluctances for airgap, magnets, teeth, and yoke
3. Solves system of equations for magnetic potentials
4. Computes flux distribution and performance metrics

Advantages: Sub-second solve time, excellent for parametric studies

### FEA Solver (High-Fidelity)

Finite Element Analysis using GetDP for accurate electromagnetic simulation:

1. Generates 2D mesh using gmsh with physical regions
2. Formulates 2D magnetostatic problem with:
   - Edge-based vector potential (Az) formulation
   - Reluctivity-based material models
   - Radial coercive field for permanent magnets
   - Dirichlet boundary conditions (A=0 at outer boundary)
3. Solves using GetDP professional solver
4. Post-processes flux density distribution

Advantages: High accuracy, captures fringing fields and saturation effects

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

## Dependencies

Core Python packages (see `requirements.txt`):
- **numpy**: Numerical computations
- **scipy**: Sparse matrix solvers
- **matplotlib**: Visualization and UI
- **gmsh**: Mesh generation for FEA
- **meshio**: Mesh I/O utilities

External Solvers:
- **GetDP**: Professional open-source electromagnetic solver
  - Companion to gmsh
  - Magnetostatic formulation
  - Robust sparse matrix solvers

## Future Enhancements

Potential additions:
- Interior permanent magnet (IPM) topologies
- 3D electromagnetic analysis
- Thermal analysis with CFD coupling
- Mechanical stress analysis
- Multi-objective optimization algorithms
- Export to CAD formats (STEP, IGES)
- Transient electromagnetic analysis
- Demagnetization analysis

## License

MIT License

## Author

Jacob Krizan
Expert in electric motor design for electric vehicles

## Acknowledgments

Built with Python, NumPy, SciPy, and Matplotlib for professional motor design analysis.

Electromagnetic solvers:
- **GetDP**: Professional open-source electromagnetic solver (https://getdp.info/)
- **gmsh**: Advanced mesh generation (https://gmsh.info/)

Special thanks to the open-source scientific computing community.
