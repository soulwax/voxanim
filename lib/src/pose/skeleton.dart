import '../math/anim_vec3.dart';
import 'bone.dart';
import 'pose.dart';

class BoneInfo {
  final String name;
  final int parentIndex; // -1 for root
  final BoneTransform bindPose;

  BoneInfo({
    required this.name,
    this.parentIndex = -1,
    BoneTransform? bindPose,
  }) : bindPose = bindPose ?? BoneTransform.identity;
}

/// Hierarchical bone skeleton managing forward kinematics.
class Skeleton {
  final List<BoneInfo> bones;
  final Map<String, int> _boneNameToIndex = {};

  Skeleton(this.bones) {
    for (var i = 0; i < bones.length; i++) {
      _boneNameToIndex[bones[i].name] = i;
    }
  }

  int? indexOf(String boneName) => _boneNameToIndex[boneName];

  /// Computes global world transforms for all bones given a local [pose].
  List<BoneTransform> evaluateGlobalPoses(Pose pose) {
    final globals = List<BoneTransform>.generate(
      bones.length,
      (_) => BoneTransform.identity,
    );

    for (var i = 0; i < bones.length; i++) {
      final info = bones[i];
      final local = pose.getBone(info.name) ?? info.bindPose;

      if (info.parentIndex < 0) {
        // Root bone
        globals[i] = local.copy();
      } else {
        final parentGlobal = globals[info.parentIndex];
        // Global position = parent.position + parent.rotation * local.position
        final globalPos = parentGlobal.position + parentGlobal.rotation.rotate(local.position);
        // Global rotation = parent.rotation * local.rotation
        final globalRot = (parentGlobal.rotation * local.rotation).normalized;
        // Global scale = parent.scale * local.scale
        final globalScale = AnimVec3(
          parentGlobal.scale.x * local.scale.x,
          parentGlobal.scale.y * local.scale.y,
          parentGlobal.scale.z * local.scale.z,
        );

        globals[i] = BoneTransform(
          position: globalPos,
          rotation: globalRot,
          scale: globalScale,
        );
      }
    }

    return globals;
  }
}
