"""ROS-independent safety and velocity arbitration primitives."""

import time


class LatchedEmergencyStop:
    """A stop that remains active until an explicit reset is requested."""

    def __init__(self):
        self.latched = False
        self.changed_at = 0.0

    def trigger(self, now=None):
        changed = not self.latched
        self.latched = True
        self.changed_at = time.monotonic() if now is None else float(now)
        return changed

    def reset(self, now=None):
        changed = self.latched
        self.latched = False
        self.changed_at = time.monotonic() if now is None else float(now)
        return changed


class VelocityArbiter:
    """Select the freshest highest-priority non-emergency velocity source."""

    def __init__(self, source_specs):
        self.sources = {
            name: {
                'priority': int(spec['priority']),
                'timeout': float(spec['timeout']),
                'message': None,
                'stamp': float('-inf'),
            }
            for name, spec in source_specs.items()
        }

    def update(self, name, message, now=None):
        source = self.sources[name]
        source['message'] = message
        source['stamp'] = time.monotonic() if now is None else float(now)

    def clear(self):
        for source in self.sources.values():
            source['message'] = None
            source['stamp'] = float('-inf')

    def select(self, now=None):
        current = time.monotonic() if now is None else float(now)
        active = []
        for name, source in self.sources.items():
            if source['message'] is None:
                continue
            if current - source['stamp'] <= source['timeout']:
                active.append((source['priority'], name, source['message']))
        if not active:
            return '', None
        _, name, message = max(active, key=lambda item: item[0])
        return name, message
