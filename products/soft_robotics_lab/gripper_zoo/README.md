# Gripper Zoo

50 pre-generated gripper designs for soft robotics research.

## Generate

```bash
python generate_zoo.py
```

## Browse

```bash
# List all designs
python browser.py

# Filter by gesture
python browser.py --gesture pinch
python browser.py --gesture power

# Filter by environment
python browser.py --env surgical
python browser.py --env wet

# Filter by object compliance
python browser.py --compliance soft

# Combine filters
python browser.py --gesture pinch --env surgical

# Show specific design
python browser.py --show GD-20260128-0001

# Statistics
python browser.py --stats
```

## Design Matrix

| Gesture | Environments | Compliances | Actuator |
|---------|--------------|-------------|----------|
| Pinch | dry, wet, surgical | rigid, medium, soft | Tendon |
| Power | dry, wet, surgical | rigid, medium, soft | Pneumatic |
| Wrap | dry, wet, surgical | rigid, medium, soft | Pneumatic |
| Hook | dry, wet, surgical | rigid, medium, soft | Tendon |
| Lateral | dry, wet, surgical | rigid, medium, soft | Tendon |
| Squeeze | dry, wet, surgical | rigid, medium, soft | Jamming |
| Spread | dry, wet, surgical | rigid, medium, soft | Pneumatic |

## Special Designs

- Surgical Micro-Gripper
- Kelp Harvester
- Industrial Pick-and-Place
- Egg Handler
- Underwater Retrieval

## Export Formats

Each design includes:

- `*.json` - Full specification
- `*.mjcf` - MuJoCo simulation
- `*.urdf` - ROS/Gazebo
- `*_usd.json` - Omniverse/Isaac Sim
