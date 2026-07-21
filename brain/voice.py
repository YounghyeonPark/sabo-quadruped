"""
RoboKitten voice — cat sounds + speech.
=======================================

Phase 0 is a **swappable stub**: each utterance is routed to ``Body.speak`` (so
a sim/hardware backend *could* play it) and emitted as a dashboard ``Event`` so
you can see what the kitten "said". No audio dependencies.

Later, a ``PiperVoice`` backend implementing the same ``Voice`` surface drops in
to synthesize real speech (Piper TTS, PLAN §5.2) and play recorded cat sounds —
without touching any caller.
"""

from __future__ import annotations

from brain.hal import Body, Event, EventSink

# Canonical cat-directed sounds (PLAN §5.1). The value is the clip id handed to
# Body.speak; the key is the caller-facing verb.
CAT_SOUNDS = {
    "trill": "friendly greeting chirp",
    "meow": "attention-seeking meow",
    "hiss": "warning hiss",
    "chirp": "curious chirp",
}


class Voice:
    """Stub voice. Logs intent + emits an event; delegates playback to the Body."""

    def __init__(self, body: Body, events: EventSink, clock):
        self._body = body
        self._events = events
        self._now = clock  # callable -> monotonic seconds

    def _utter(self, clip: str, text: str) -> None:
        self._body.speak(clip)
        self._events.emit(Event(t=self._now(), kind="voice", text=text,
                                extra={"clip": clip}))

    # -- cat-directed sounds ----------------------------------------
    def trill(self) -> None:
        self._utter("trill", "🐱 trills a friendly hello")

    def meow(self) -> None:
        self._utter("meow", "🐱 meows for attention")

    def chirp(self) -> None:
        self._utter("chirp", "🐱 chirps curiously")

    def hiss(self) -> None:
        self._utter("hiss", "🙀 hisses a warning")

    # -- owner-directed speech (Piper later) ------------------------
    def say(self, text: str) -> None:
        """Speak a phrase to the human. Stub logs it; PiperVoice will synthesize."""
        self._utter(f"tts:{text}", f'🗣️ says: "{text}"')
