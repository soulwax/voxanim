import '../pose/pose.dart';
import 'blend_node.dart';

typedef PoseSampler = Pose Function(double time);

/// Animation clip node sampling a time-parameterized pose generator.
class ClipNode extends BlendNode {
  final String name;
  final double duration;
  final bool loop;
  final double speed;
  final PoseSampler sampler;

  ClipNode({
    required this.name,
    required this.duration,
    required this.sampler,
    this.loop = true,
    this.speed = 1.0,
  });

  @override
  Pose evaluate(double time, Map<String, double> parameters) {
    if (duration <= 0.0) return sampler(0.0);

    var t = time * speed;
    if (loop) {
      t = t % duration;
      if (t < 0.0) t += duration;
    } else {
      t = t.clamp(0.0, duration);
    }

    return sampler(t);
  }
}
