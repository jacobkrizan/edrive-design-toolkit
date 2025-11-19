"""
Electric Motor Design Toolkit - Main Application
Professional UI for permanent magnet motor design and analysis
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

from motor_parameters import MotorParameters, PerformanceMetrics
from geometry import MotorGeometry
from magnetic_circuit import MagneticCircuitSolver
from visualization import MotorVisualizer


class MotorDesignApp:
    """Main application class for motor design toolkit"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("E-Drive Design Toolkit - Permanent Magnet Motor Designer")
        self.root.geometry("1600x900")
        
        # Initialize motor parameters
        self.params = MotorParameters()
        self.geometry = None
        self.solver = None
        self.visualizer = None
        self.metrics = None
        
        # Rotor position for animation
        self.rotor_angle = 0.0
        
        # Create UI
        self.create_ui()
        
        # Initial calculation
        self.update_design()
    
    def create_ui(self):
        """Create the user interface"""
        # Main container
        main_container = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Controls
        left_panel = ttk.Frame(main_container, width=350)
        main_container.add(left_panel, weight=0)
        
        # Right panel - Visualization
        right_panel = ttk.Frame(main_container)
        main_container.add(right_panel, weight=1)
        
        # Create control panel
        self.create_control_panel(left_panel)
        
        # Create visualization panel
        self.create_visualization_panel(right_panel)
    
    def create_control_panel(self, parent):
        """Create parameter controls"""
        # Title
        title = ttk.Label(parent, text="Motor Parameters", font=('Arial', 14, 'bold'))
        title.pack(pady=10)
        
        # Scrollable frame for parameters
        canvas = tk.Canvas(parent, width=330)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Parameter storage
        self.param_vars = {}
        
        # Basic Configuration
        self.create_param_section(scrollable_frame, "Basic Configuration", [
            ("Number of Poles", "num_poles", 4, 20, 2),
            ("Number of Slots", "num_slots", 12, 96, 6),
        ])
        
        # Geometric Parameters
        self.create_param_section(scrollable_frame, "Geometry (mm)", [
            ("Stator Outer Dia.", "stator_outer_diameter", 100, 400, 200),
            ("Stator Inner Dia.", "stator_inner_diameter", 80, 300, 120),
            ("Rotor Outer Dia.", "rotor_outer_diameter", 70, 290, 118),
            ("Rotor Inner Dia.", "rotor_inner_diameter", 20, 150, 40),
            ("Airgap Length", "airgap_length", 0.5, 5.0, 1.0),
            ("Stack Length", "stack_length", 50, 300, 100),
            ("Magnet Thickness", "magnet_thickness", 2, 15, 5),
            ("Slot Depth", "slot_depth", 10, 50, 30),
        ])
        
        # Electrical Parameters
        self.create_param_section(scrollable_frame, "Electrical", [
            ("Rated Current (A)", "rated_current", 10, 500, 100),
            ("Rated Speed (rpm)", "rated_speed", 500, 10000, 3000),
            ("Turns per Coil", "turns_per_coil", 5, 100, 20),
        ])
        
        # Material Properties
        self.create_param_section(scrollable_frame, "Materials", [
            ("Magnet Br (T)", "magnet_br", 0.8, 1.5, 1.2),
            ("Steel µr", "steel_mu_r", 500, 5000, 2000),
            ("Magnet Arc Ratio", "magnet_arc_ratio", 0.5, 1.0, 0.85),
        ])
        
        # Update button
        update_btn = ttk.Button(scrollable_frame, text="Update Design", 
                               command=self.update_design, style='Accent.TButton')
        update_btn.pack(pady=15, padx=10, fill=tk.X)
        
        # Animation controls
        anim_frame = ttk.LabelFrame(scrollable_frame, text="Animation", padding=10)
        anim_frame.pack(pady=10, padx=10, fill=tk.X)
        
        self.angle_var = tk.DoubleVar(value=0)
        angle_slider = ttk.Scale(anim_frame, from_=0, to=360, orient=tk.HORIZONTAL,
                                variable=self.angle_var, command=self.update_rotor_angle)
        angle_slider.pack(fill=tk.X, pady=5)
        
        angle_label = ttk.Label(anim_frame, text="Rotor Angle: 0°")
        angle_label.pack()
        self.angle_label = angle_label
        
        # Pack scrollbar and canvas
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Results display
        results_frame = ttk.LabelFrame(parent, text="Performance Metrics", padding=5)
        results_frame.pack(pady=10, padx=5, fill=tk.BOTH, expand=True)
        
        self.results_text = scrolledtext.ScrolledText(results_frame, height=15, width=40,
                                                      font=('Courier', 9))
        self.results_text.pack(fill=tk.BOTH, expand=True)
    
    def create_param_section(self, parent, title, params):
        """Create a section of parameters"""
        frame = ttk.LabelFrame(parent, text=title, padding=10)
        frame.pack(pady=5, padx=10, fill=tk.X)
        
        for label, key, min_val, max_val, default in params:
            param_frame = ttk.Frame(frame)
            param_frame.pack(fill=tk.X, pady=3)
            
            ttk.Label(param_frame, text=label, width=20).pack(side=tk.LEFT)
            
            var = tk.DoubleVar(value=default)
            self.param_vars[key] = var
            
            # Entry
            entry = ttk.Entry(param_frame, textvariable=var, width=10)
            entry.pack(side=tk.RIGHT, padx=5)
            
            # Slider
            if max_val - min_val <= 50:
                slider = ttk.Scale(param_frame, from_=min_val, to=max_val,
                                 orient=tk.HORIZONTAL, variable=var)
                slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)
    
    def create_visualization_panel(self, parent):
        """Create visualization area"""
        # Create matplotlib figure
        self.fig = Figure(figsize=(12, 8), dpi=100)
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Toolbar
        from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
        toolbar = NavigationToolbar2Tk(self.canvas, parent)
        toolbar.update()
    
    def update_design(self):
        """Update motor design based on current parameters"""
        # Update parameters from UI
        for key, var in self.param_vars.items():
            setattr(self.params, key, var.get())
        
        # Recalculate derived parameters
        self.params.__post_init__()
        
        # Recreate geometry and solver
        self.geometry = MotorGeometry(self.params)
        self.solver = MagneticCircuitSolver(self.params, self.geometry)
        self.visualizer = MotorVisualizer(self.params, self.geometry, self.solver)
        
        # Calculate performance
        self.metrics = self.solver.calculate_performance(self.rotor_angle)
        
        # Update visualization
        self.visualizer.create_full_visualization(self.fig, self.rotor_angle)
        self.canvas.draw()
        
        # Update results text
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(1.0, self.metrics.format_output())
    
    def update_rotor_angle(self, value):
        """Update rotor position"""
        self.rotor_angle = float(value) * np.pi / 180.0
        self.angle_label.config(text=f"Rotor Angle: {float(value):.1f}°")
        
        if self.visualizer:
            # Recalculate and update
            self.metrics = self.solver.calculate_performance(self.rotor_angle)
            self.visualizer.create_full_visualization(self.fig, self.rotor_angle)
            self.canvas.draw()
            
            # Update results
            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(1.0, self.metrics.format_output())


def main():
    """Main entry point"""
    root = tk.Tk()
    
    # Configure style
    style = ttk.Style()
    style.theme_use('clam')
    
    app = MotorDesignApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
