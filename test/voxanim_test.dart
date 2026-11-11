import 'dart:math' as math;
import 'package:test/test.dart';
import 'package:voxanim/voxanim.dart';

void main() {
  group('voxanim: Math Primitives', () {
    test('AnimVec3 arithmetic, cross, and distance', () {
      const a = AnimVec3(1.0, 2.0, 3.0);
      const b = AnimVec3(4.0, 5.0, 6.0);

      expect(a.dot(b), equals(32.0));
      expect(a.cross(b), equals(const AnimVec3(-3.0, 6.0, -3.0)));
      expect(a.distance(a), equals(0.0));
      expect(AnimVec3.zero.distance(const AnimVec3(0.0, 3.0, 4.0)), equals(5.0));
    });

    test('AnimQuat fromAxisAngle, rotate, and slerp', () {
      final q90 = AnimQuat.fromAxisAngle(AnimVec3.up, math.pi * 0.5); // 90 deg around Y
      final rotated = q90.rotate(const AnimVec3(1.0, 0.0, 0.0));

      expect(rotated.x, closeTo(0.0, 1e-6));
      expect(rotated.y, closeTo(0.0, 1e-6));
      expect(rotated.z, closeTo(-1.0, 1e-6));

      // Slerp halfway (45 deg)
      final q45 = AnimQuat.slerp(AnimQuat.identity, q90, 0.5);
      final rotated45 = q45.rotate(const AnimVec3(1.0, 0.0, 0.0));
      expect(rotated45.x, closeTo(math.sqrt(0.5), 1e-6));
      expect(rotated45.z, closeTo(-math.sqrt(0.5), 1e-6));
    });
  });

  group('voxanim: Poses & Forward Kinematics', () {
    test('Pose.lerp blends bone transforms accurately', () {
      final poseA = Pose({
        'Root': BoneTransform(position: const AnimVec3(0.0, 0.0, 0.0)),
      });
      final poseB = Pose({
        'Root': BoneTransform(position: const AnimVec3(10.0, 20.0, 30.0)),
      });

      final blended = Pose.lerp(poseA, poseB, 0.5);
      expect(blended.getBone('Root')!.position, equals(const AnimVec3(5.0, 10.0, 15.0)));
    });

    test('Pose.applyAdditive layers delta animations on top of base pose', () {
      final base = Pose({
        'Chest': BoneTransform(position: const AnimVec3(0.0, 1.0, 0.0)),
      });
      final delta = Pose({
        'Chest': BoneTransform(position: const AnimVec3(0.0, 0.2, 0.0)), // Breathing delta
      });

      base.applyAdditive(delta, 0.5);
      expect(base.getBone('Chest')!.position.y, closeTo(1.1, 1e-6));
    });

    test('Skeleton forward kinematics propagates global transforms down hierarchy', () {
      final skeleton = Skeleton([
        BoneInfo(name: 'Hip', parentIndex: -1),
        BoneInfo(name: 'Spine', parentIndex: 0),
        BoneInfo(name: 'Head', parentIndex: 1),
      ]);

      final localPose = Pose({
        'Hip': BoneTransform(position: const AnimVec3(0.0, 1.0, 0.0)),
        'Spine': BoneTransform(position: const AnimVec3(0.0, 0.5, 0.0)),
        'Head': BoneTransform(position: const AnimVec3(0.0, 0.3, 0.0)),
      });

      final globals = skeleton.evaluateGlobalPoses(localPose);

      expect(globals[0].position, equals(const AnimVec3(0.0, 1.0, 0.0)));
      expect(globals[1].position, equals(const AnimVec3(0.0, 1.5, 0.0)));
      expect(globals[2].position, equals(const AnimVec3(0.0, 1.8, 0.0)));
    });
  });

  group('voxanim: Blend Trees', () {
    test('Blend1DNode interpolates between thresholds based on scalar parameter', () {
      final idlePose = Pose({'Root': BoneTransform(position: const AnimVec3(0.0, 0.0, 0.0))});
      final walkPose = Pose({'Root': BoneTransform(position: const AnimVec3(2.0, 0.0, 0.0))});
      final runPose = Pose({'Root': BoneTransform(position: const AnimVec3(6.0, 0.0, 0.0))});

      final blendTree = Blend1DNode(
        parameterName: 'speed',
        entries: [
          Blend1DEntry(threshold: 0.0, node: ClipNode(name: 'Idle', duration: 1.0, sampler: (_) => idlePose)),
          Blend1DEntry(threshold: 2.0, node: ClipNode(name: 'Walk', duration: 1.0, sampler: (_) => walkPose)),
          Blend1DEntry(threshold: 6.0, node: ClipNode(name: 'Run', duration: 1.0, sampler: (_) => runPose)),
        ],
      );

      // Speed 0.0 -> Idle
      final p0 = blendTree.evaluate(0.0, {'speed': 0.0});
      expect(p0.getBone('Root')!.position.x, equals(0.0));

      // Speed 1.0 -> 50% between Idle and Walk
      final p1 = blendTree.evaluate(0.0, {'speed': 1.0});
      expect(p1.getBone('Root')!.position.x, closeTo(1.0, 1e-6));

      // Speed 4.0 -> 50% between Walk and Run
      final p4 = blendTree.evaluate(0.0, {'speed': 4.0});
      expect(p4.getBone('Root')!.position.x, closeTo(4.0, 1e-6));
    });

    test('Blend2DNode blends multi-way along 2D parameters', () {
      final centerPose = Pose({'Root': BoneTransform(position: const AnimVec3(0.0, 0.0, 0.0))});
      final forwardPose = Pose({'Root': BoneTransform(position: const AnimVec3(0.0, 0.0, 5.0))});
      final rightPose = Pose({'Root': BoneTransform(position: const AnimVec3(5.0, 0.0, 0.0))});

      final blend2D = Blend2DNode(
        parameterX: 'moveX',
        parameterY: 'moveZ',
        entries: [
          Blend2DEntry(posX: 0.0, posY: 0.0, node: ClipNode(name: 'Center', duration: 1.0, sampler: (_) => centerPose)),
          Blend2DEntry(posX: 0.0, posY: 1.0, node: ClipNode(name: 'Forward', duration: 1.0, sampler: (_) => forwardPose)),
          Blend2DEntry(posX: 1.0, posY: 0.0, node: ClipNode(name: 'Right', duration: 1.0, sampler: (_) => rightPose)),
        ],
      );

      // Exact center
      final pCenter = blend2D.evaluate(0.0, {'moveX': 0.0, 'moveZ': 0.0});
      expect(pCenter.getBone('Root')!.position.length, closeTo(0.0, 1e-6));

      // Diagonal (moveX: 0.5, moveZ: 0.5)
      final pDiag = blend2D.evaluate(0.0, {'moveX': 0.5, 'moveZ': 0.5});
      expect(pDiag.getBone('Root')!.position.x, greaterThan(0.0));
      expect(pDiag.getBone('Root')!.position.z, greaterThan(0.0));
    });
  });

  group('voxanim: Inverse Kinematics Solvers', () {
    test('TwoBoneIkSolver reaches target and preserves bone segment lengths', () {
      const root = AnimVec3(0.0, 2.0, 0.0);
      const mid = AnimVec3(0.0, 1.0, 0.0);
      const end = AnimVec3(0.0, 0.0, 0.0);
      const target = AnimVec3(0.5, 0.3, 0.0);
      const pole = AnimVec3(0.0, 1.0, 1.0); // Bend knee forward (+Z)

      final result = TwoBoneIkSolver.solve(
        root: root,
        mid: mid,
        end: end,
        target: target,
        poleTarget: pole,
      );

      // Length preservation
      final lenA = result.rootPosition.distance(result.midPosition);
      final lenB = result.midPosition.distance(result.endPosition);
      expect(lenA, closeTo(1.0, 1e-3));
      expect(lenB, closeTo(1.0, 1e-3));

      // End effector reaches target
      expect(result.endPosition.distance(target), closeTo(0.0, 1e-3));

      // Knee bent towards pole direction (+Z)
      expect(result.midPosition.z, greaterThan(0.0));
    });

    test('FabrikSolver iteratively reaches target across 4-joint chain', () {
      final chain = [
        const AnimVec3(0.0, 0.0, 0.0),
        const AnimVec3(0.0, 1.0, 0.0),
        const AnimVec3(0.0, 2.0, 0.0),
        const AnimVec3(0.0, 3.0, 0.0),
      ];
      const target = AnimVec3(1.5, 1.5, 0.0);

      final solved = FabrikSolver.solve(
        jointPositions: chain,
        target: target,
      );

      expect(solved, hasLength(4));
      // Root fixed at origin
      expect(solved.first, equals(AnimVec3.zero));
      // End effector reaches target within tolerance
      expect(solved.last.distance(target), closeTo(0.0, 0.01));

      // Segment lengths preserved
      for (var i = 0; i < solved.length - 1; i++) {
        expect(solved[i].distance(solved[i + 1]), closeTo(1.0, 0.01));
      }
    });

    test('AimIkSolver orients forward vector towards target with cone limit', () {
      const bonePos = AnimVec3(0.0, 1.5, 0.0);
      const currentRot = AnimQuat.identity;
      const target = AnimVec3(0.0, 1.5, 10.0); // Directly in front (+Z)

      final aimRot = AimIkSolver.solve(
        bonePosition: bonePos,
        currentRotation: currentRot,
        targetPosition: target,
        localAimAxis: AnimVec3.forward,
      );

      final aimDir = aimRot.rotate(AnimVec3.forward);
      expect(aimDir.z, closeTo(1.0, 1e-3));
      expect(aimDir.x.abs(), lessThan(1e-3));
    });
  });
}
