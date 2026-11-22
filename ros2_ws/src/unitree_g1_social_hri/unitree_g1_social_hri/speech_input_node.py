"""speech_input_node - microphone (or keyboard) -> text on /hri/speech_text.

Modes (parameter ``input_mode``):
  text  : type commands in this terminal (Project A, no microphone needed)
  voice : microphone + offline speech-to-text (Project B)
            stt_engine:=vosk    streaming, small CPU model (default)
            stt_engine:=whisper energy-gated recording + openai-whisper
  none  : publish nothing (use `ros2 topic pub` or another terminal)

Safety: stop words ("stop", "freeze", "halt", "emergency") raise the emergency
stop *locally* on /hri/emergency_stop, without waiting for any language model.
In text mode, type ``/resume`` to release it (a deliberate human action).
"""
from __future__ import annotations

import json
import queue
import re
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from . import hri_messages as hm

STOP_PATTERN = re.compile(r"\b(stop|freeze|halt|emergency)\b", re.I)


class SpeechInputNode(Node):
    def __init__(self):
        super().__init__("speech_input_node")
        self.declare_parameter("input_mode", "text")
        self.declare_parameter("stt_engine", "vosk")
        self.declare_parameter("vosk_model_path", "")
        self.declare_parameter("whisper_model", "base.en")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("energy_threshold", 0.02)
        self.declare_parameter("max_utterance_s", 6.0)
        self.mode = self.get_parameter("input_mode").value
        self.pub_text = self.create_publisher(String, hm.TOPICS["speech_text"], 10)
        self.pub_stop = self.create_publisher(Bool, hm.TOPICS["emergency_stop"], 10)
        self._running = True
        target = {"text": self._text_loop, "voice": self._voice_loop}.get(self.mode)
        if target:
            self.thread = threading.Thread(target=target, daemon=True)
            self.thread.start()
        self.get_logger().info(f"speech_input_node ready (input_mode={self.mode})")

    # ------------------------------------------------------------------ output
    def publish_text(self, text: str, source: str) -> None:
        text = text.strip()
        if not text:
            return
        if STOP_PATTERN.search(text):
            self.pub_stop.publish(Bool(data=True))
            self.get_logger().warn("Stop word heard -> EMERGENCY STOP raised")
        msg = {"id": hm.new_id(), "text": text, "source": source, "t_input": hm.now()}
        self.pub_text.publish(String(data=hm.encode(msg)))
        self.get_logger().info(f"[{source}] \"{text}\" (id={msg['id']})")

    # -------------------------------------------------------------------- text
    def _text_loop(self) -> None:
        print("\nType a command for the G1 (e.g. 'Hello G1, please wave your right hand').\n"
              "  /stop = emergency stop, /resume = release it, /quit = exit\n", flush=True)
        while self._running and rclpy.ok():
            try:
                line = input("you> ")
            except (EOFError, KeyboardInterrupt):
                break
            cmd = line.strip().lower()
            if cmd == "/quit":
                break
            if cmd == "/stop":
                self.pub_stop.publish(Bool(data=True))
                continue
            if cmd == "/resume":
                self.pub_stop.publish(Bool(data=False))
                self.get_logger().info("Emergency stop released by operator")
                continue
            self.publish_text(line, "keyboard")

    # ------------------------------------------------------------------- voice
    def _voice_loop(self) -> None:
        engine = self.get_parameter("stt_engine").value
        try:
            if engine == "whisper":
                self._whisper_loop()
            else:
                self._vosk_loop()
        except Exception as e:  # missing microphone / model / package
            self.get_logger().error(
                f"Voice input failed ({type(e).__name__}: {e}). "
                "Check the microphone (python3 -m sounddevice) and that the STT model is "
                "downloaded (scripts/download_models.sh), or use input_mode:=text.")

    def _vosk_loop(self) -> None:
        import sounddevice as sd
        from vosk import KaldiRecognizer, Model

        path = self.get_parameter("vosk_model_path").value
        if not path:
            raise FileNotFoundError("vosk_model_path parameter is empty")
        rate = int(self.get_parameter("sample_rate").value)
        rec = KaldiRecognizer(Model(path), rate)
        audio_q: "queue.Queue[bytes]" = queue.Queue()

        def callback(indata, frames, time_info, status):  # runs in audio thread
            audio_q.put(bytes(indata))

        self.get_logger().info("Listening (Vosk)... speak a command")
        with sd.RawInputStream(samplerate=rate, blocksize=4000, dtype="int16",
                               channels=1, callback=callback):
            while self._running and rclpy.ok():
                if rec.AcceptWaveform(audio_q.get()):
                    text = json.loads(rec.Result()).get("text", "")
                    self.publish_text(text, "vosk")

    def _whisper_loop(self) -> None:
        import numpy as np
        import sounddevice as sd
        import whisper

        rate = 16000  # whisper expects 16 kHz mono float32
        model = whisper.load_model(self.get_parameter("whisper_model").value)
        thr = float(self.get_parameter("energy_threshold").value)
        max_s = float(self.get_parameter("max_utterance_s").value)
        block = int(0.1 * rate)
        self.get_logger().info("Listening (Whisper)... speak a command")
        with sd.InputStream(samplerate=rate, channels=1, dtype="float32", blocksize=block) as stream:
            while self._running and rclpy.ok():
                chunk, _ = stream.read(block)
                if float(np.sqrt(np.mean(chunk ** 2))) < thr:
                    continue
                frames, silent = [chunk], 0
                while len(frames) * 0.1 < max_s and silent < 8:   # stop after 0.8 s silence
                    chunk, _ = stream.read(block)
                    frames.append(chunk)
                    silent = silent + 1 if float(np.sqrt(np.mean(chunk ** 2))) < thr else 0
                audio = np.concatenate(frames)[:, 0]
                result = model.transcribe(audio, fp16=False, language="en")
                self.publish_text(result.get("text", ""), "whisper")

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechInputNode()
    try:
        if node.mode == "text":
            # keep ROS spinning while the keyboard thread runs; exit on /quit
            while rclpy.ok() and node.thread.is_alive():
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
