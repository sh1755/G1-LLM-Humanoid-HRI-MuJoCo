"""speech_output_node - /hri/speech_output -> robot voice.

Parameter ``tts_backend``:
  print   : print to the terminal only (default, always works)
  espeak  : espeak-ng command-line synthesiser (sudo apt install espeak-ng)
  piper   : Piper neural TTS (pip install piper-tts; needs a .onnx voice, see
            scripts/download_models.sh) played with aplay
  pyttsx3 : pyttsx3 Python package (uses espeak-ng on Linux)
Any failure falls back to printing, so the demo never stalls on audio.
"""
from __future__ import annotations

import queue
import shutil
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import hri_messages as hm


class SpeechOutputNode(Node):
    def __init__(self):
        super().__init__("speech_output_node")
        self.declare_parameter("tts_backend", "print")
        self.declare_parameter("piper_voice", "")
        self.backend = self.get_parameter("tts_backend").value
        self.create_subscription(String, hm.TOPICS["speech_output"], self.on_speech, 10)
        self.jobs: "queue.Queue[str]" = queue.Queue(maxsize=10)
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f"speech_output_node ready (tts_backend={self.backend})")

    def on_speech(self, msg: String) -> None:
        text = hm.decode(msg.data).get("text", "")
        if text:
            self.get_logger().info(f"G1 says: \"{text}\"")
            try:
                self.jobs.put_nowait(text)
            except queue.Full:
                pass

    def _worker(self) -> None:
        while rclpy.ok():
            try:
                text = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.speak(text)
            except Exception as e:
                self.get_logger().warn(f"TTS '{self.backend}' failed ({e}); text shown above")

    def speak(self, text: str) -> None:
        if self.backend == "espeak":
            exe = shutil.which("espeak-ng") or shutil.which("espeak")
            if not exe:
                raise RuntimeError("espeak-ng not installed")
            subprocess.run([exe, "-s", "160", text], check=True, timeout=30)
        elif self.backend == "piper":
            voice = self.get_parameter("piper_voice").value
            if not voice:
                raise RuntimeError("piper_voice parameter is empty")
            with tempfile.NamedTemporaryFile(suffix=".wav") as wav:
                subprocess.run(["python3", "-m", "piper", "-m", voice, "-f", wav.name, "--", text],
                               check=True, timeout=60)
                subprocess.run(["aplay", "-q", wav.name], check=True, timeout=60)
        elif self.backend == "pyttsx3":
            import pyttsx3
            engine = pyttsx3.init()
            engine.say(text)
            engine.runAndWait()
        # "print": already logged in on_speech


def main(args=None):
    rclpy.init(args=args)
    node = SpeechOutputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
