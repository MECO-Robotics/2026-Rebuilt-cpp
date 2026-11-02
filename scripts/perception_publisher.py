"""Low-batching publisher for the robot's existing camera-space detection protocol.

Import from a coprocessor pipeline after each completed inference. Camera points
must use metres, X forward/Y left/Z up. latency_seconds is capture-to-publication
age measured in the producer's own clock domain, not a clock offset to the robot.
"""
import math
import ntcore


class DetectionPublisher:
    def __init__(self, instance, camera):
        if camera not in ("front", "rear"):
            raise ValueError("camera must be front or rear")
        self.instance = instance
        self.sequence = 0
        self.publisher = instance.getDoubleArrayTopic(
            f"/Perception/{camera}/detections"
        ).publish(ntcore.PubSubOptions(periodic=0.005))

    def publish(self, detections, latency_seconds):
        if not math.isfinite(latency_seconds) or latency_seconds < 0:
            raise ValueError("capture-to-publication latency must be finite and nonnegative")
        rows = [tuple(row) for row in detections]
        if len(rows) > 128 or any(
            len(row) != 4 or not all(math.isfinite(x) for x in row)
            or not 0 <= row[3] <= 1 for row in rows
        ):
            raise ValueError("expected at most 128 finite (x,y,z,confidence) detections")
        self.sequence += 1
        self.publisher.set([1, self.sequence, latency_seconds, *(x for row in rows for x in row)])
        self.instance.flush()

    def close(self):
        self.publisher.close()
