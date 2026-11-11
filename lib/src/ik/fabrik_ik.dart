import '../math/anim_vec3.dart';

/// Multi-joint Inverse Kinematics solver using the FABRIK algorithm.
class FabrikSolver {
  /// Solves an N-joint chain reaching towards [target].
  /// Returns the solved positions list.
  static List<AnimVec3> solve({
    required List<AnimVec3> jointPositions,
    required AnimVec3 target,
    double tolerance = 1e-3,
    int maxIterations = 15,
  }) {
    if (jointPositions.length < 2) return List.of(jointPositions);

    final n = jointPositions.length;
    final points = List.of(jointPositions);

    // Compute segment lengths
    final lengths = <double>[];
    var totalLength = 0.0;
    for (var i = 0; i < n - 1; i++) {
      final len = points[i].distance(points[i + 1]);
      lengths.add(len);
      totalLength += len;
    }

    final root = points.first;
    final distToTarget = root.distance(target);

    // Case 1: Target unreachable (stretched straight towards target)
    if (distToTarget > totalLength) {
      final dir = (target - root).normalized;
      for (var i = 0; i < n - 1; i++) {
        points[i + 1] = points[i] + dir * lengths[i];
      }
      return points;
    }

    // Case 2: Target reachable -> Forward and Backward iterations
    for (var iter = 0; iter < maxIterations; iter++) {
      // Check if end effector is close enough to target
      if (points.last.distance(target) <= tolerance) {
        break;
      }

      // Backward reaching: start from end at target, move backward to root
      points[n - 1] = target;
      for (var i = n - 2; i >= 0; i--) {
        final dir = (points[i] - points[i + 1]).normalized;
        points[i] = points[i + 1] + dir * lengths[i];
      }

      // Forward reaching: set root back to origin, move forward to end
      points[0] = root;
      for (var i = 0; i < n - 1; i++) {
        final dir = (points[i + 1] - points[i]).normalized;
        points[i + 1] = points[i] + dir * lengths[i];
      }
    }

    return points;
  }
}
