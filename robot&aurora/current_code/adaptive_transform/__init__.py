"""
Adaptive Transform Package - Coordinate transformation system with spatial region division.

This package provides tools for coordinate transformation between different reference frames
using region-based adaptive transformation matrices.
"""

from .core import AdaptiveTransform
from .utils import (
    build_coordinate_transformation_model,
    transform_pose,
    print_transformation_results,
    find_transformation,
    estimate_transform_matrix,
    transformation_error,
    validate_transformation,
    rotation_matrix_to_euler,
    euler_to_rotation_matrix,
    test_transformation_functions
)
from .main import main

# Package metadata
__version__ = "1.0.0"
__author__ = "Your Name"
__email__ = "your.email@example.com"

# Public API
__all__ = [
    'AdaptiveTransform',
    'build_coordinate_transformation_model',
    'transform_pose',
    'print_transformation_results',
    'find_transformation',
    'estimate_transform_matrix',
    'transformation_error',
    'validate_transformation',
    'rotation_matrix_to_euler',
    'euler_to_rotation_matrix',
    'test_transformation_functions',
    'main'
]

# Package-level constants
DEFAULT_DIVISIONS = 2
DEFAULT_EPSILON = 5

# Quick usage example (can be imported as adaptive_transform.example)
def example():
    """
    Example usage of the adaptive transform package.
    """
    print("Adaptive Transform Package Example:")
    print("1. Import the package: import adaptive_transform")
    print("2. Use main function: adaptive_transform.main(...)")
    print("3. Or use individual components:")
    print("   - adaptive_transform.AdaptiveTransform(...)")
    print("   - adaptive_transform.build_coordinate_transformation_model(...)")
    print("   - adaptive_transform.transform_pose(...)")
    
    print("\nFor detailed usage, see documentation or main.py")


# Package initialization message (optional, can be removed in production)
def _show_init_message():
    """Show package initialization message."""
    print(f"Adaptive Transform Package v{__version__} loaded successfully")
    print("Use adaptive_transform.example() for usage examples")

# Uncomment the following line if you want to show init message on import
# _show_init_message()