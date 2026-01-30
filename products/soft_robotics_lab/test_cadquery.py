"""Verbose CadQuery test - creates a box and exports to STL."""
import sys
print("Python:", sys.executable)
print("---")

print("Importing cadquery...")
try:
    import cadquery as cq
    print("CadQuery imported OK")
    print("CadQuery version:", getattr(cq, "__version__", "unknown"))
except ImportError as e:
    print("CadQuery import failed (ImportError):", e)
    sys.exit(1)
except Exception as e:
    print("CadQuery import failed:", type(e).__name__, e)
    sys.exit(1)

print("Creating box...")
try:
    box = cq.Workplane("XY").box(10, 10, 10)
    print("Box created")
except Exception as e:
    print("Box creation failed:", e)
    sys.exit(1)

print("Exporting to test_box.stl...")
try:
    cq.exporters.export(box, "test_box.stl")
    print("Done - check for test_box.stl")
except Exception as e:
    print("Export failed:", e)
    sys.exit(1)
