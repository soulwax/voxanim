import 'dart:math' as math;
import '../math/anim_vec3.dart';
import '../math/anim_quat.dart';

/// Aim / LookAt Inverse Kinematics solver directing a bone towards a 3D target.
class AimIkSolver {
  /// Computes the aim rotation for a bone at [bonePosition] with [currentRotation],
  /// aligning its [localAimAxis] towards [targetPosition].
  static AnimQuat solve({
    required AnimVec3 bonePosition,
    required AnimQuat currentRotation,
    required AnimVec3 targetPosition,
    AnimVec3 localAimAxis = AnimVec3.forward,
    double maxAngleRadians = math.pi * 0.5, // 90 degree cone
    double weight = 1.0,
  }) {
    if (weight <= 0.0) return currentRotation;

    final toTarget = targetPosition - bonePosition;
    if (toTarget.lengthSquared < 1e-6) return currentRotation;

    final targetDir = toTarget.normalized;
    final currentAimDir = currentRotation.rotate(localAimAxis).normalized;

    // Deflection angle between current aim and target
    var angle = math.acos(currentAimDir.dot(targetDir).clamp(-1.0, 1.0));

    // Clamp angle to cone limit
    if (angle > maxAngleRadians) {
      angle = maxAngleRadians;
    }

    var rotAxis = currentAimDir.cross(targetDir).normalized;
    if (rotAxis.lengthSquared < 1e-6) {
      rotAxis = AnimVec3.up;
    }

    final aimDelta = AnimQuat.fromAxisAngle(rotAxis, angle * weight.clamp(0.0, 1.0));
    return (aimDelta * currentRotation).normalized;
  }
}
