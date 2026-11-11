import 'dart:math' as math;
import '../math/anim_vec3.dart';
import '../math/anim_quat.dart';

class TwoBoneIkResult {
  final AnimVec3 rootPosition;
  final AnimVec3 midPosition;
  final AnimVec3 endPosition;
  final AnimQuat rootRotationDelta;
  final AnimQuat midRotationDelta;

  const TwoBoneIkResult({
    required this.rootPosition,
    required this.midPosition,
    required this.endPosition,
    required this.rootRotationDelta,
    required this.midRotationDelta,
  });
}

/// Analytical 2-bone Inverse Kinematics solver (law of cosines with pole target).
class TwoBoneIkSolver {
  /// Solves 2-bone IK placing [end] at [target] with knee/elbow bending towards [poleTarget].
  static TwoBoneIkResult solve({
    required AnimVec3 root,
    required AnimVec3 mid,
    required AnimVec3 end,
    required AnimVec3 target,
    required AnimVec3 poleTarget,
  }) {
    final lengthA = root.distance(mid);
    final lengthB = mid.distance(end);
    final maxReach = lengthA + lengthB;
    final minReach = (lengthA - lengthB).abs();

    final toTarget = target - root;
    var targetDist = toTarget.length;

    // Clamp distance to reachable range
    if (targetDist > maxReach - 1e-4) {
      targetDist = maxReach - 1e-4;
    } else if (targetDist < minReach + 1e-4) {
      targetDist = minReach + 1e-4;
    }

    final targetDir = toTarget.normalized;

    // Law of cosines: compute angle at root (alpha) and angle at mid (beta)
    final cosAlpha = ((lengthA * lengthA + targetDist * targetDist - lengthB * lengthB) /
            (2.0 * lengthA * targetDist))
        .clamp(-1.0, 1.0);
    final sinAlpha = math.sqrt(1.0 - cosAlpha * cosAlpha);

    // Bend plane normal via pole vector
    final toPole = poleTarget - root;
    var bendNormal = toTarget.cross(toPole).normalized;
    if (bendNormal.lengthSquared < 1e-6) {
      // Degenerate pole: construct arbitrary orthogonal vector
      bendNormal = targetDir.cross(AnimVec3.up).normalized;
      if (bendNormal.lengthSquared < 1e-6) {
        bendNormal = targetDir.cross(AnimVec3.right).normalized;
      }
    }

    // Direction from root to mid joint on the bend plane
    final bendDir = bendNormal.cross(targetDir).normalized;
    final midDir = (targetDir * cosAlpha + bendDir * sinAlpha).normalized;
    final solvedMid = root + midDir * lengthA;

    // End effector placed at target distance along targetDir, or reaching towards target
    final solvedEnd = root + targetDir * targetDist;

    // Compute rotation deltas
    final origRootToMid = (mid - root).normalized;
    final origMidToEnd = (end - mid).normalized;

    final newRootToMid = (solvedMid - root).normalized;
    final newMidToEnd = (solvedEnd - solvedMid).normalized;

    final rootRotDelta = AnimQuat.fromToRotation(origRootToMid, newRootToMid);
    final midRotDelta = AnimQuat.fromToRotation(origMidToEnd, newMidToEnd);

    return TwoBoneIkResult(
      rootPosition: root,
      midPosition: solvedMid,
      endPosition: solvedEnd,
      rootRotationDelta: rootRotDelta,
      midRotationDelta: midRotDelta,
    );
  }
}
