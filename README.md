# Sustav za korekciju putanje industrijskog robota u zavarivanju primjenom laserskog vizijskog sustava

**Diplomski rad** (Master's Thesis)

A laser vision system for industrial robot weld-path correction in welding applications.

---

## Repository Structure

```
├── fanuc_karel/   # FANUC Karel programs for the welding robot
├── camera/        # Laser vision / camera acquisition and processing code
└── analysis/      # Data analysis, evaluation scripts, and thesis figures
```

### `fanuc_karel/`
FANUC Karel source programs that run on the robot controller. Includes motion programs,
path correction routines, and communication with the vision system.

### `camera/`
Code for the laser vision sensor: image acquisition, laser-line detection, 3D reconstruction,
and weld-seam feature extraction.

### `analysis/`
Offline post-processing scripts and notebooks: statistical evaluation of path corrections,
visualisation plots, and comparison of corrected vs. uncorrected weld paths.
