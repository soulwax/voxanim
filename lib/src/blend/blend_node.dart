import '../pose/pose.dart';

/// Abstract animation graph node that evaluates into a [Pose].
abstract class BlendNode {
  /// Evaluates the node given current [time] in seconds and graph [parameters].
  Pose evaluate(double time, Map<String, double> parameters);
}
