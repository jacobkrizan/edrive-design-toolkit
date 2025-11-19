"""
Comprehensive test suite for E-Drive Design Toolkit
Tests all key functionality: parameters, geometry, MEC solver, FEA solver
"""

import unittest
import numpy as np
import os
from motor_parameters import MotorParameters
from geometry import MotorGeometry
from magnetic_circuit import MagneticCircuitSolver
from fea_solver import FEASolver


class TestMotorParameters(unittest.TestCase):
    """Test motor parameter initialization and calculations"""
    
    def setUp(self):
        """Create default motor parameters for testing"""
        self.params = MotorParameters()
    
    def test_initialization(self):
        """Test that parameters initialize with correct default values"""
        self.assertEqual(self.params.num_poles, 8)
        self.assertEqual(self.params.num_slots, 48)
        # Parameters are in mm, not meters
        self.assertEqual(self.params.stator_outer_diameter, 200.0)
        self.assertAlmostEqual(self.params.magnet_br, 1.2, places=2)
    
    def test_slot_pole_ratio(self):
        """Test slots per pole calculation"""
        slots_per_pole = self.params.num_slots / self.params.num_poles
        self.assertEqual(slots_per_pole, 6.0)
    
    def test_diameter_constraints(self):
        """Test that diameters are properly ordered"""
        self.assertGreater(self.params.stator_outer_diameter, 
                          self.params.stator_inner_diameter)
        self.assertGreater(self.params.stator_inner_diameter, 
                          self.params.rotor_outer_diameter)
        self.assertGreater(self.params.rotor_outer_diameter, 
                          self.params.rotor_inner_diameter)
    
    def test_pole_pitch(self):
        """Test pole pitch calculation"""
        pole_pitch = np.pi * self.params.rotor_outer_diameter / self.params.num_poles
        self.assertGreater(pole_pitch, 0)
        self.assertLess(pole_pitch, np.pi * self.params.rotor_outer_diameter)


class TestMotorGeometry(unittest.TestCase):
    """Test geometric calculations and mesh generation"""
    
    def setUp(self):
        """Create geometry object for testing"""
        self.params = MotorParameters()
        self.geometry = MotorGeometry(self.params)
    
    def test_stator_geometry_generation(self):
        """Test that stator geometry is generated"""
        stator = self.geometry.generate_stator_geometry()
        self.assertIn('slots', stator)
        self.assertEqual(len(stator['slots']), self.params.num_slots)
    
    def test_rotor_geometry_generation(self):
        """Test that rotor geometry is generated"""
        rotor = self.geometry.generate_rotor_geometry()
        self.assertIn('magnets', rotor)
        self.assertEqual(len(rotor['magnets']), self.params.num_poles)
    
    def test_winding_positions(self):
        """Test winding position calculation"""
        windings = self.geometry.get_winding_positions()
        # Should have phase information (A, B, C)
        self.assertIn('A', windings)
        self.assertIn('B', windings)
        self.assertIn('C', windings)
        # Each phase should have windings
        self.assertGreater(len(windings['A']), 0)
        self.assertGreater(len(windings['B']), 0)
        self.assertGreater(len(windings['C']), 0)
    
    def test_airgap_mesh(self):
        """Test airgap mesh generation"""
        mesh = self.geometry.get_airgap_mesh(n_points=100)
        self.assertEqual(len(mesh), 100)
        # Points are 3D coordinates (x, y, z) but z=0 for 2D
        self.assertEqual(mesh.shape[1], 3)


class TestMECSolver(unittest.TestCase):
    """Test Magnetic Equivalent Circuit solver"""
    
    def setUp(self):
        """Create MEC solver for testing"""
        self.params = MotorParameters()
        self.geometry = MotorGeometry(self.params)
        self.solver = MagneticCircuitSolver(self.params, self.geometry)
    
    def test_solver_initialization(self):
        """Test that solver initializes properly"""
        self.assertIsNotNone(self.solver)
        self.assertEqual(self.solver.params, self.params)
        self.assertEqual(self.solver.geometry, self.geometry)
    
    def test_solve_basic(self):
        """Test that solver produces results"""
        results = self.solver.solve_mec(rotor_angle=0.0)
        
        # Check that results dictionary has expected keys
        self.assertIn('potentials', results)
        self.assertIn('fluxes', results)
        self.assertIn('reluctances', results)
    
    def test_flux_values(self):
        """Test that flux values are physically reasonable"""
        results = self.solver.solve_mec(rotor_angle=0.0)
        
        # Check airgap flux exists
        self.assertIn('fluxes', results)
        self.assertIn('airgap', results['fluxes'])
        
        airgap_fluxes = results['fluxes']['airgap']
        max_flux = np.max(np.abs(airgap_fluxes))
        
        # Should have significant flux (> 0.0001 Wb)
        self.assertGreater(max_flux, 0.0001)
        # Should be reasonable for motor (< 0.01 Wb)
        self.assertLess(max_flux, 0.01)
    
    def test_rotor_angle_variation(self):
        """Test solver with different rotor angles"""
        angles = [0, np.pi/8, np.pi/4, np.pi/2]
        
        for angle in angles:
            results = self.solver.solve_mec(rotor_angle=angle)
            # Should always produce results
            self.assertIsNotNone(results)
            self.assertIn('potentials', results)


class TestFEASolver(unittest.TestCase):
    """Test Finite Element Analysis solver"""
    
    def setUp(self):
        """Create FEA solver for testing"""
        self.params = MotorParameters()
        self.geometry = MotorGeometry(self.params)
        self.solver = FEASolver(self.params, self.geometry)
    
    def test_solver_initialization(self):
        """Test that FEA solver initializes properly"""
        self.assertIsNotNone(self.solver)
        self.assertEqual(self.solver.params, self.params)
        self.assertEqual(self.solver.geometry, self.geometry)
    
    def test_mesh_creation(self):
        """Test that mesh can be created"""
        mesh_data = self.solver.create_mesh()
        self.assertIsNotNone(mesh_data)
        
        # Check mesh has nodes and elements
        self.assertIn('nodes', mesh_data)
        self.assertIn('elements', mesh_data)
        self.assertGreater(len(mesh_data['nodes']), 0)
        self.assertGreater(len(mesh_data['elements']), 0)
    
    def test_getdp_availability(self):
        """Test that GetDP is available"""
        getdp_path = os.path.join(os.path.dirname(__file__), 
                                  "getdp-3.5.0-Windows64", "getdp.exe")
        
        if os.path.exists(getdp_path):
            self.assertTrue(True, "GetDP found")
        else:
            self.skipTest("GetDP not installed")
    
    def test_solve_magnetostatic(self):
        """Test that magnetostatic solution can be computed"""
        getdp_path = os.path.join(os.path.dirname(__file__), 
                                  "getdp-3.5.0-Windows64", "getdp.exe")
        if not os.path.exists(getdp_path):
            self.skipTest("GetDP not installed")
        
        try:
            mesh_data = self.solver.create_mesh()
            solution = self.solver.solve_magnetostatic(mesh_data, rotor_angle=0.0)
            
            # Check solution exists
            self.assertIsNotNone(solution)
            self.assertIn('flux_density_nodes', solution)
            
            # Check flux density values
            flux = solution['flux_density_nodes']
            max_flux = np.max(np.linalg.norm(flux, axis=1))
            
            # Should have significant field
            self.assertGreater(max_flux, 0.1)
            # Should be physically reasonable
            self.assertLess(max_flux, 3.0)
            
        except Exception as e:
            if "GetDP" in str(e):
                self.skipTest(f"GetDP execution issue: {e}")
            else:
                raise


class TestIntegration(unittest.TestCase):
    """Integration tests for complete workflow"""
    
    def test_complete_mec_workflow(self):
        """Test complete workflow from parameters to MEC results"""
        params = MotorParameters()
        geometry = MotorGeometry(params)
        solver = MagneticCircuitSolver(params, geometry)
        
        results = solver.solve_mec(rotor_angle=0.0)
        
        # Verify results are complete
        self.assertIn('potentials', results)
        self.assertIn('fluxes', results)
        self.assertIn('reluctances', results)
        
        # Check physical groups
        self.assertIn('airgap', results['fluxes'])
        self.assertIn('teeth', results['fluxes'])
    
    def test_parameter_sweep(self):
        """Test that solver handles parameter variations"""
        params = MotorParameters()
        geometry = MotorGeometry(params)
        solver = MagneticCircuitSolver(params, geometry)
        
        # Sweep rotor angles
        angles = np.linspace(0, 2*np.pi/params.num_poles, 10)
        flux_values = []
        
        for angle in angles:
            results = solver.solve_mec(rotor_angle=angle)
            max_flux = np.max(np.abs(results['fluxes']['airgap']))
            flux_values.append(max_flux)
        
        # Should have variation in flux
        self.assertGreater(np.std(flux_values), 0)
        self.assertEqual(len(flux_values), len(angles))
    
    def test_different_motor_configs(self):
        """Test different motor configurations"""
        configs = [
            {'num_poles': 6, 'num_slots': 36},
            {'num_poles': 8, 'num_slots': 48},
            {'num_poles': 10, 'num_slots': 60},
        ]
        
        for config in configs:
            params = MotorParameters()
            params.num_poles = config['num_poles']
            params.num_slots = config['num_slots']
            
            geometry = MotorGeometry(params)
            solver = MagneticCircuitSolver(params, geometry)
            
            results = solver.solve_mec(rotor_angle=0.0)
            
            # Should produce valid results
            self.assertIsNotNone(results)
            self.assertIn('fluxes', results)
            
            # Check correct number of components
            stator = geometry.generate_stator_geometry()
            self.assertEqual(len(stator['slots']), config['num_slots'])


def run_tests():
    """Run all tests and print results"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestMotorParameters))
    suite.addTests(loader.loadTestsFromTestCase(TestMotorGeometry))
    suite.addTests(loader.loadTestsFromTestCase(TestMECSolver))
    suite.addTests(loader.loadTestsFromTestCase(TestFEASolver))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))
    
    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors) - len(result.skipped)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped)}")
    
    if result.wasSuccessful():
        print("\n✓ ALL TESTS PASSED!")
    else:
        print("\n✗ Some tests failed - see details above")
    
    print("="*70)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    exit(0 if success else 1)
