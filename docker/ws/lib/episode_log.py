"""An episode, on disk, with the two timestamps every observation actually has.

An episode is what a policy experiment is compared on, so the file has to answer questions the
topic list cannot: which frame was this, when was it *acquired* versus when did we *receive* it,
how old was each arm's own state when it was used, and what was commanded versus what was measured.

The layout is deliberately boring — two JSONL files and a manifest, no rosbag2:

    episodes/<episode>/manifest.json      what it is, the clocks, the counts, the caveats
    episodes/<episode>/observations.jsonl one line per sample: topic, stamp, receipt, age, payload
    episodes/<episode>/commands.jsonl     one line per command the supervisor accepted
    episodes/<episode>/frames/*.jpg       the actual pixels the policy would see

Why not rosbag2: the episode's authority is the *pair* (frame, named arm state at a known age),
and a bag records whatever the graph published, in publish order, with no per-arm age and nothing
saying which image the policy was given. A bag stays useful for inspection; this file is the
record of the decision.

`EpisodeLog` is stdlib only and takes its clocks as arguments, so `tools/test_episode_recorder.py`
pins the arithmetic — ages, sampling, the manifest counts — with no ROS and no cameras.
"""

import json
import os
import time

# A sample older than this was already useless when it was recorded; it is kept and counted, so a
# run with a dead camera cannot look healthy in the manifest.
STALE_S = 0.5


def iso(wall):
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(wall)) + ".%03d" % int((wall % 1) * 1000)


class EpisodeLog:
    def __init__(self, root, instruction="", clock=time.time, mono=time.monotonic, stale_s=STALE_S):
        self.clock, self.mono, self.stale_s = clock, mono, stale_s
        self.started_wall, self.started_mono = clock(), mono()
        name = time.strftime("%Y%m%d-%H%M%S", time.localtime(self.started_wall))
        self.dir = os.path.join(root, name)
        n = 1
        while os.path.exists(self.dir):          # two episodes in one second must not share a dir
            n += 1
            self.dir = os.path.join(root, "%s-%d" % (name, n))
        os.makedirs(os.path.join(self.dir, "frames"), exist_ok=True)
        self.manifest = {
            "episode": os.path.basename(self.dir),
            "instruction": instruction,
            "started": iso(self.started_wall), "ended": None,
            "duration_s": None,
            "clocks": {"wall": "time.time()", "mono": "time.monotonic()",
                       "note": "mono is the interval clock; every age below is mono - stamp"},
            "observations": {},          # topic -> count
            "stale": {},                 # topic -> count of samples older than stale_s
            "max_age_s": {},             # topic -> oldest sample seen
            "commands": 0,
            "jaw": {"unit": "percent (LeRobot)", "tf_map": "see deg2rad.JAW_RAD_PER_PERCENT",
                    "calibrated": False,
                    "note": "the jaw angle in TF is a drawing until a measured aperture fits it"},
            "open_questions": [],
        }
        self._obs = open(os.path.join(self.dir, "observations.jsonl"), "w", encoding="utf-8")
        self._cmd = open(os.path.join(self.dir, "commands.jsonl"), "w", encoding="utf-8")
        self._frames = 0

    # ------------------------------------------------------------------ observations
    def _write(self, fh, row):
        fh.write(json.dumps(row, sort_keys=True) + "\n")

    def note(self, text):
        """A caveat that belongs in the manifest, e.g. 'scene extrinsics not usable'."""
        if text not in self.manifest["open_questions"]:
            self.manifest["open_questions"].append(text)

    def observe(self, topic, stamp_wall=None, payload=None, frame=None, received_wall=None):
        """Record one sample.

        `stamp_wall` is the message's own acquisition stamp (None if it carries none) and
        `received_wall` is when the node saw it. The age must be measured at **receipt**, not at
        write: with the files written on another thread, a queued frame written 1 s later is not a
        frame acquired 1 s late, and mixing the two made a healthy run report 1.27 s of age."""
        now_wall, now_mono = self.clock(), self.mono()
        ref = received_wall if received_wall is not None else now_wall
        age = None if stamp_wall is None else round(ref - stamp_wall, 3)
        row = {"topic": topic, "received": round(now_mono, 4), "stamp": stamp_wall,
               "age_s": age, "mono": round(now_mono, 4),
               "queued_s": round(now_wall - ref, 3) if received_wall is not None else 0.0}
        if payload is not None:
            row["data"] = payload
        if frame is not None:
            name = "frames/%06d.jpg" % self._frames
            self._frames += 1
            with open(os.path.join(self.dir, name), "wb") as fh:
                fh.write(frame)
            row["file"] = name
        self._write(self._obs, row)
        self.manifest["observations"][topic] = self.manifest["observations"].get(topic, 0) + 1
        if age is not None:
            if age > self.stale_s:
                self.manifest["stale"][topic] = self.manifest["stale"].get(topic, 0) + 1
            if age > self.manifest["max_age_s"].get(topic, -1.0):
                self.manifest["max_age_s"][topic] = age
        return row

    def command(self, source, values, accepted=True, reason=None):
        """A command as the supervisor accepted it — the requested numbers, and why it refused
        anything it did not accept. Measured values live in the observations."""
        row = {"at": round(self.mono(), 4), "source": source, "accepted": bool(accepted),
               "values": values, "reason": reason}
        self._write(self._cmd, row)
        if accepted:
            self.manifest["commands"] += 1
        return row

    def flush(self, ended=False):
        """Write the manifest now, atomically.

        Called periodically and NOT only at close: a recorder ends because the container is
        stopped, and `docker stop` sends SIGKILL after its grace period — measured on the board,
        an episode that was killed left 705 frames and 1604 observations with **no manifest at
        all**. A file that only exists if the process exits politely is not a record."""
        if ended:
            self.manifest["ended"] = iso(self.clock())
            self.manifest["duration_s"] = round(self.mono() - self.started_mono, 3)
        tmp = os.path.join(self.dir, "manifest.json.tmp")
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(self.manifest, fh, indent=2, sort_keys=True)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, os.path.join(self.dir, "manifest.json"))

    def close(self):
        self._obs.close()
        self._cmd.close()
        self.flush(ended=True)
        return self.dir

    def summary(self):
        """One line for the log: what an operator needs to know the episode is worth keeping."""
        arms = {t: n for t, n in self.manifest["observations"].items() if "joint_states" in t}
        # every camera topic in this cell ends in the same JPEG suffix (the scene one has no "image"
        # in its name, which is why filtering on that word lost it)
        imgs = {t.rsplit("/", 1)[0].lstrip("/"): n
                for t, n in self.manifest["observations"].items()
                if t.rsplit("/", 1)[-1] == "compressed"}
        worst = max(self.manifest["max_age_s"].values()) if self.manifest["max_age_s"] else 0.0
        return ("%s: %d frames, arms %s, images %s, %d commands, worst age %.2fs%s"
                % (self.manifest["episode"], self._frames, arms, imgs, self.manifest["commands"],
                   worst, "" if not self.manifest["stale"] else
                   " — STALE samples: %s" % self.manifest["stale"]))
