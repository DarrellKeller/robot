"""One owner for microphone, Whisper and blocking TTS playback."""
from __future__ import annotations

import queue
import logging
import re
import threading
import time
from collections import Counter

from robot_schemas import TranscriptQuality

WAKE_WORDS = ("robot", "mauricio", "maurice", "spinny", "toad")


def transcript_quality(result):
    words = re.findall(r"\w+", result.get("text", "").lower())
    triples = Counter(tuple(words[i:i + 3]) for i in range(max(0, len(words) - 2)))
    segments = result.get("segments", [])
    return TranscriptQuality(**{"word_count": len(words), "max_repeated_trigram": max(triples.values(), default=0),
            "no_speech_probability": max((s.get("no_speech_prob", 0) for s in segments), default=0),
            "compression_ratio": max((s.get("compression_ratio", 0) for s in segments), default=0),
            "average_log_probability": min((s.get("avg_logprob", 0) for s in segments), default=0)}).model_dump()


class AudioController:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.events = queue.Queue()
        self.jobs = queue.Queue(maxsize=1)
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.state = "idle" if enabled else "disabled"
        self.revision = 0
        self.thread = None
        if enabled:
            self.thread = threading.Thread(target=self._run, daemon=True, name="audio")
            self.thread.start()

    def status(self):
        with self.lock:
            return self.state

    def _state(self, value):
        with self.lock:
            self.state = value

    def invalidate(self, revision):
        with self.lock:
            self.revision = revision

    def speak(self, text, ask, revision):
        if not self.enabled or self.jobs.full():
            return False
        self.jobs.put_nowait((text, ask, revision))
        return True

    def listen(self, revision):
        if not self.enabled or self.jobs.full():
            return False
        self.jobs.put_nowait((None, False, revision))
        return True

    def _capture(self, interface, pyaudio, np, command=False):
        # The microphone is closed before any speaker playback begins.
        stream = interface.open(format=pyaudio.paInt16, channels=1, rate=16000,
                                input=True, frames_per_buffer=4000)
        chunks = []
        started = time.monotonic()
        voiced = False
        silent_since = None
        try:
            while not self.closed.is_set():
                data = np.frombuffer(stream.read(4000, exception_on_overflow=False), dtype=np.int16).astype(np.float32) / 32768
                chunks.append(data)
                if float(np.sqrt(np.mean(data * data))) > 0.02:
                    voiced, silent_since = True, None
                elif voiced and silent_since is None:
                    silent_since = time.monotonic()
                elapsed = time.monotonic() - started
                if not command and (elapsed >= 2 or not self.jobs.empty()):
                    break
                if command and (elapsed >= 15 or (silent_since and time.monotonic() - silent_since >= 1.5)):
                    break
        finally:
            stream.stop_stream()
            stream.close()
        return np.concatenate(chunks) if chunks and voiced else None

    def _handle_wake(self, text, tts_ready, tts_module, quality=None):
        match = re.search(r'\b(' + '|'.join(WAKE_WORDS) + r')\b', text, re.I)
        if not match:
            return False
        logging.info("Wake word detected: %s", match.group())
        # Stop before acknowledging; capture has already closed the microphone.
        # This acknowledgement must never complete a mission's talk step.
        self._state("listening")
        self.events.put({"kind": "listening"})
        if not (tts_ready and tts_module.speak("Huh?")):
            logging.warning("Wake acknowledgement playback failed")
        remainder = text[match.end():].strip(' ,.!?')
        if remainder:
            self.events.put({"kind": "user", "text": remainder, "quality": quality or {}})
            return False
        return True

    def _run(self):
        interface = None
        try:
            import numpy as np
            import pyaudio
            import mlx_whisper
            from lfm_tools import LOCAL_INFERENCE_LOCK
            import tts_module
            tts_ready = tts_module.initialize_tts()
            interface = pyaudio.PyAudio()
            listen_next = False
            while not self.closed.is_set():
                try:
                    text, ask, revision = self.jobs.get_nowait()
                except queue.Empty:
                    pass
                else:
                    with self.lock:
                        valid = revision == self.revision
                    if valid:
                        if text is None:
                            listen_next = True
                            self.events.put({"kind": "listening"})
                            continue
                        self._state("talking")
                        success = tts_ready and tts_module.speak(text)
                        self.events.put({"kind": "spoken" if success else "speech_failed",
                                         "text": text, "ask": ask, "revision": revision})
                        listen_next = bool(ask and success)
                        if listen_next:
                            self.events.put({"kind": "listening"})
                    continue
                self._state("listening" if listen_next else "wake_listening")
                captured = self._capture(interface, pyaudio, np, command=listen_next)
                was_command = listen_next
                listen_next = False
                if captured is None:
                    if was_command:
                        self.events.put({"kind": "listen_timeout"})
                    continue
                self._state("transcribing" if was_command else "wake_transcribing")
                with LOCAL_INFERENCE_LOCK:
                    result = mlx_whisper.transcribe(captured,
                        path_or_hf_repo="mlx-community/whisper-base.en-mlx", language="en",
                        condition_on_previous_text=False)
                text = result.get("text", "").strip()
                quality = transcript_quality(result)
                if was_command:
                    self.events.put({"kind": "user" if text else "listen_timeout", "text": text, "quality": quality})
                    continue
                listen_next = self._handle_wake(text, tts_ready, tts_module, quality)
        except Exception as exc:
            self.events.put({"kind": "audio_error", "error": type(exc).__name__})
        finally:
            self._state("disabled")
            if interface:
                interface.terminate()

    def close(self):
        self.closed.set()
        if self.thread:
            self.thread.join(timeout=1)
