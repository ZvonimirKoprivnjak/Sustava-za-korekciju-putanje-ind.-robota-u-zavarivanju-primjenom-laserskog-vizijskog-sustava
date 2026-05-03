# Sustav za korekciju putanje industrijskog robota u zavarivanju primjenom laserskog vizijskog sustava

**Diplomski rad** (Master's Thesis)

A laser vision system for industrial robot weld-path correction in welding applications.

---

## Repository Structure

```
├── fanuc_karel/   # FANUC Karel programs
├── vision system/        # Laser vision / camera acquisition and processing code
└── analysis/      # Data analysiss
```

### `fanuc_karel/`
FANUC Karel source programs that run on the robot controller. Includes motion programs and communication with the vision system.

### `camera/`
Code for the laser vision sensor: image acquisition, laser-line detection, weld-seam feature extraction 
and comunication with FANUC industirla robot

### `analysis/`
Offline post-processing scripts: statistical evaluation of path corrections,
visualisation plots, and comparison of corrected vs. reference weld paths.
