import 'dart:math' as math;
import 'anim_vec3.dart';

/// 3D rotation quaternion for bones, joint orientations, and IK solvers.
class AnimQuat {
  final double x;
  final double y;
  final double z;
  final double w;

  const AnimQuat(this.x, this.y, this.z, this.w);

  static const AnimQuat identity = AnimQuat(0.0, 0.0, 0.0, 1.0);

  factory AnimQuat.fromAxisAngle(AnimVec3 axis, double radians) {
    final half = radians * 0.5;
    final sinHalf = math.sin(half);
    final normAxis = axis.normalized;
    return AnimQuat(
      normAxis.x * sinHalf,
      normAxis.y * sinHalf,
      normAxis.z * sinHalf,
      math.cos(half),
    );
  }

  factory AnimQuat.fromToRotation(AnimVec3 from, AnimVec3 to) {
    final v1 = from.normalized;
    final v2 = to.normalized;
    final dot = v1.dot(v2);

    if (dot >= 1.0 - 1e-6) {
      return AnimQuat.identity;
    }
    if (dot <= -1.0 + 1e-6) {
      // 180 degree rotation around arbitrary orthogonal axis
      var axis = AnimVec3.right.cross(v1);
      if (axis.lengthSquared < 1e-6) {
        axis = AnimVec3.up.cross(v1);
      }
      return AnimQuat.fromAxisAngle(axis.normalized, math.pi);
    }

    final axis = v1.cross(v2);
    final q = AnimQuat(axis.x, axis.y, axis.z, 1.0 + dot);
    return q.normalized;
  }

  factory AnimQuat.lookRotation(AnimVec3 forward, [AnimVec3 up = AnimVec3.up]) {
    final f = forward.normalized;
    if (f.lengthSquared < 1e-12) return AnimQuat.identity;

    var r = up.cross(f).normalized;
    if (r.lengthSquared < 1e-12) {
      r = AnimVec3.right.cross(f).normalized;
    }
    final u = f.cross(r);

    // Matrix to quaternion conversion
    final m00 = r.x;
    final m01 = u.x;
    final m02 = f.x;
    final m10 = r.y;
    final m11 = u.y;
    final m12 = f.y;
    final m20 = r.z;
    final m21 = u.z;
    final m22 = f.z;

    final trace = m00 + m11 + m22;
    if (trace > 0.0) {
      final s = 0.5 / math.sqrt(trace + 1.0);
      return AnimQuat(
        (m21 - m12) * s,
        (m02 - m20) * s,
        (m10 - m01) * s,
        0.25 / s,
      );
    } else if (m00 > m11 && m00 > m22) {
      final s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22);
      return AnimQuat(
        0.25 * s,
        (m01 + m10) / s,
        (m02 + m20) / s,
        (m21 - m12) / s,
      );
    } else if (m11 > m22) {
      final s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22);
      return AnimQuat(
        (m01 + m10) / s,
        0.25 * s,
        (m12 + m21) / s,
        (m02 - m20) / s,
      );
    } else {
      final s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11);
      return AnimQuat(
        (m02 + m20) / s,
        (m12 + m21) / s,
        0.25 * s,
        (m10 - m01) / s,
      );
    }
  }

  AnimQuat get inverse => AnimQuat(-x, -y, -z, w);

  double get lengthSquared => x * x + y * y + z * z + w * w;
  double get length => math.sqrt(lengthSquared);

  AnimQuat get normalized {
    final len = length;
    if (len < 1e-12) return AnimQuat.identity;
    return AnimQuat(x / len, y / len, z / len, w / len);
  }

  AnimQuat operator *(AnimQuat rhs) {
    return AnimQuat(
      w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
      w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
      w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
      w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
    );
  }

  AnimVec3 rotate(AnimVec3 v) {
    final qv = AnimVec3(x, y, z);
    final uv = qv.cross(v);
    final uuv = qv.cross(uv);
    return v + (uv * (2.0 * w)) + (uuv * 2.0);
  }

  static AnimQuat slerp(AnimQuat a, AnimQuat b, double t) {
    var cosHalfTheta = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    var bx = b.x;
    var by = b.y;
    var bz = b.z;
    var bw = b.w;

    if (cosHalfTheta < 0.0) {
      cosHalfTheta = -cosHalfTheta;
      bx = -bx;
      by = -by;
      bz = -bz;
      bw = -bw;
    }

    if (cosHalfTheta >= 1.0 - 1e-6) {
      return AnimQuat(
        a.x + (bx - a.x) * t,
        a.y + (by - a.y) * t,
        a.z + (bz - a.z) * t,
        a.w + (bw - a.w) * t,
      ).normalized;
    }

    final halfTheta = math.acos(cosHalfTheta);
    final sinHalfTheta = math.sqrt(1.0 - cosHalfTheta * cosHalfTheta);

    if (sinHalfTheta.abs() < 1e-6) {
      return a;
    }

    final ratioA = math.sin((1.0 - t) * halfTheta) / sinHalfTheta;
    final ratioB = math.sin(t * halfTheta) / sinHalfTheta;

    return AnimQuat(
      a.x * ratioA + bx * ratioB,
      a.y * ratioA + by * ratioB,
      a.z * ratioA + bz * ratioB,
      a.w * ratioA + bw * ratioB,
    );
  }

  @override
  String toString() => 'AnimQuat(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)}, ${z.toStringAsFixed(3)}, ${w.toStringAsFixed(3)})';
}
