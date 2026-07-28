"""Temporal tracking and stable primary-target selection."""

import config


class BallTracker:
    def __init__(self, frame_width=320, frame_height=320):
        self.tracks = []
        self._next_track_id = 1
        self._control_track_id = None
        self._frame_width = float(frame_width)
        self._frame_height = float(frame_height)

    @staticmethod
    def candidate_from_detection(detection):
        score = float(detection.score)
        width = int(detection.w)
        height = int(detection.h)
        if score < config.CANDIDATE_CONFIDENCE_THRESHOLD:
            return None
        if width < config.MIN_BOX_SIDE or height < config.MIN_BOX_SIDE:
            return None

        aspect_ratio = width / float(height)
        if not (
            config.MIN_ASPECT_RATIO
            <= aspect_ratio
            <= config.MAX_ASPECT_RATIO
        ):
            return None

        x = int(detection.x)
        y = int(detection.y)
        return {
            "x": float(x),
            "y": float(y),
            "w": float(width),
            "h": float(height),
            "cx": x + width * 0.5,
            "cy": y + height * 0.5,
            "score": score,
        }

    def update(self, candidates):
        unmatched_tracks = set(range(len(self.tracks)))
        matched_candidates = set()
        possible_matches = []

        for track_index, track in enumerate(self.tracks):
            for candidate_index, candidate in enumerate(candidates):
                size_ratio = max(
                    candidate["w"] / max(1.0, track["w"]),
                    track["w"] / max(1.0, candidate["w"]),
                    candidate["h"] / max(1.0, track["h"]),
                    track["h"] / max(1.0, candidate["h"]),
                )
                if size_ratio > 1.8:
                    continue

                dx = candidate["cx"] - track["cx"]
                dy = candidate["cy"] - track["cy"]
                distance_squared = dx * dx + dy * dy
                match_distance = max(
                    10.0,
                    config.TRACK_MATCH_DISTANCE_FACTOR
                    * max(
                        candidate["w"],
                        candidate["h"],
                        track["w"],
                        track["h"],
                    ),
                )
                if distance_squared > match_distance * match_distance:
                    continue
                possible_matches.append(
                    (distance_squared, track_index, candidate_index)
                )

        for _, track_index, candidate_index in sorted(possible_matches):
            if track_index not in unmatched_tracks:
                continue
            if candidate_index in matched_candidates:
                continue

            track = self.tracks[track_index]
            candidate = candidates[candidate_index]
            keep = 1.0 - config.TRACK_SMOOTHING
            for key in ("x", "y", "w", "h", "cx", "cy", "score"):
                track[key] = (
                    keep * track[key]
                    + config.TRACK_SMOOTHING * candidate[key]
                )
            track["hits"] += 1
            track["misses"] = 0
            unmatched_tracks.remove(track_index)
            matched_candidates.add(candidate_index)

        for track_index in unmatched_tracks:
            track = self.tracks[track_index]
            track["misses"] += 1
            track["hits"] = max(0, track["hits"] - 1)

        for candidate_index, candidate in enumerate(candidates):
            if candidate_index in matched_candidates:
                continue
            candidate["id"] = self._next_track_id
            candidate["hits"] = 1
            candidate["misses"] = 0
            self._next_track_id += 1
            self.tracks.append(candidate)

        self.tracks[:] = [
            track
            for track in self.tracks
            if track["misses"] <= config.TRACK_MAX_MISSES
        ]

        displayed = [
            track
            for track in self.tracks
            if (
                track["misses"] == 0
                and track["hits"] >= config.TRACK_CONFIRM_FRAMES
                and track["score"]
                >= config.DISPLAY_CONFIDENCE_THRESHOLD
            )
        ]
        controllable = [
            track
            for track in displayed
            if (
                track["hits"] >= config.CONTROL_CONFIRM_FRAMES
                and track["score"]
                >= config.CONTROL_CONFIDENCE_THRESHOLD
            )
        ]
        selected = self._select_primary(controllable)
        return displayed, controllable, selected

    def _select_primary(self, tracks):
        if not tracks:
            self._control_track_id = None
            return None

        if self._control_track_id is not None:
            for track in tracks:
                if track["id"] == self._control_track_id:
                    return track

        selected = max(tracks, key=self._priority)
        self._control_track_id = selected["id"]
        return selected

    def select_warning(self, displayed):
        """Choose one red-box observation for early vehicle slowdown."""
        if not displayed:
            return None
        return max(displayed, key=self._priority)

    def _priority(self, track):
        frame_area = self._frame_width * self._frame_height
        area_score = min(
            1.0,
            track["w"] * track["h"] / (0.08 * frame_area),
        )
        half_width = max(1.0, self._frame_width * 0.5)
        center_score = max(
            0.0,
            1.0 - abs(track["cx"] - half_width) / half_width,
        )
        bottom_score = max(
            0.0,
            min(1.0, track["cy"] / max(1.0, self._frame_height)),
        )
        return (
            0.50 * track["score"]
            + 0.25 * area_score
            + 0.15 * center_score
            + 0.10 * bottom_score
        )
