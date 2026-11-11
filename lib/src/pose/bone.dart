import '../math/anim_vec3.dart';
import '../math/anim_quat.dart';

/// Represents local position, rotation, and scale for a single skeletal bone or joint.
class BoneTransform {
  AnimVec3 position;
  AnimQuat rotation;
  AnimVec3 scale;

  BoneTransform({
    this.position = AnimVec3.zero,
    this.rotation = AnimQuat.identity,
    this.scale = AnimVec3.one,
  });

  static BoneTransform get identity => BoneTransform();

  BoneTransform copy() => BoneTransform(
        position: position,
        rotation: rotation,
        scale: scale,
      );

  BoneTransform lerp(BoneTransform other, double t) {
    return BoneTransform(
      position: position.lerp(other.position, t),
      rotation: AnimQuat.slerp(rotation, other.rotation, t),
      scale: scale.lerp(other.scale, t),
    );
  }

  @override
  String toString() => 'BoneTransform(pos: $position, rot: $rotation, scl: $scale)';
}
