# voxanim

High-performance skeletal animation blending, 1D/2D blend trees, and inverse kinematics (TwoBoneIK, FABRIK, AimIK) for Dart.

## Features

- **Inverse Kinematics (IK)**:
  - **TwoBoneIK**: Closed-form analytical law-of-cosines solver with pole target for realistic knee and elbow bending.
  - **FABRIK**: Multi-joint forward and backward reaching solver for tentacles, ropes, spines, and multi-segment limbs.
  - **AimIK / LookAtIK**: Directs bone orientation towards 3D targets with cone angle clamping and weight blending.
- **Animation Blend Trees**:
  - **1D Blend Tree**: Smooth interpolation across scalar parameters (e.g. speed: Idle -> Walk -> Run).
  - **2D Blend Tree**: Multi-directional Cartesian blending across 2D velocity parameters.
  - **Additive Blending**: Layering secondary delta motions (breathing, flinching, recoil) on top of base poses.
- **Skeletal Poses & Forward Kinematics**:
  - `BoneTransform`, `Pose` (with linear blending and additive layers), and `Skeleton` for computing global world-space transforms from hierarchical local poses.
- **Pure Dart**: Zero platform dependencies, VM-first, runs on Web, Flutter, and Native.

## Example

```dart
import 'package:voxanim/voxanim.dart';

void main() {
  final ikResult = TwoBoneIkSolver.solve(
    root: AnimVec3(0, 2, 0),
    mid: AnimVec3(0, 1, 0),
    end: AnimVec3(0, 0, 0),
    target: AnimVec3(0.5, 0.2, 0.0),
    poleTarget: AnimVec3(0, 1, 1),
  );

  print('Solved Mid (Knee): ${ikResult.midPosition}');
  print('Solved End (Foot): ${ikResult.endPosition}');
}
```
