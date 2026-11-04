# audio2text.py
# Install first: pip install SpeechRecognition pydub
# For mp3/m4a/other formats, pydub also needs ffmpeg installed on your system.

import os
import sqlite3
import sys

import speech_recognition as sr
from pydub import AudioSegment

DB_FILE = "transcripts.db"


def init_db():
    con = sqlite3.connect(DB_FILE)
    con.execute("""
        CREATE TABLE IF NOT EXISTS transcripts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            audio_file TEXT,
            text TEXT,
            binary TEXT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)
    con.commit()
    return con


def to_wav(path):
    """Convert any audio format to WAV, which the recognizer needs."""
    if path.lower().endswith(".wav"):
        return path
    wav_path = os.path.splitext(path)[0] + "_converted.wav"
    AudioSegment.from_file(path).export(wav_path, format="wav")
    return wav_path


def audio_to_text(path):
    recognizer = sr.Recognizer()
    with sr.AudioFile(to_wav(path)) as source:
        audio = recognizer.record(source)
    return recognizer.recognize_google(audio)  # requires internet


def text_to_binary(text):
    """Convert text to its binary (machine-level) form, one byte per group."""
    return " ".join(format(byte, "08b") for byte in text.encode("utf-8"))


def main(path):
    if not os.path.exists(path):
        sys.exit(f"File not found: {path}")

    text = audio_to_text(path)
    binary = text_to_binary(text)

    con = init_db()
    con.execute(
        "INSERT INTO transcripts (audio_file, text, binary) VALUES (?, ?, ?)",
        (path, text, binary),
    )
    con.commit()
    con.close()

    txt_path = os.path.splitext(path)[0] + ".txt"
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(text)

    print("Text:  ", text)
    print("Binary:", binary[:100] + ("..." if len(binary) > 100 else ""))
    print(f"Saved to {DB_FILE} and {txt_path}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Usage: python audio2text.py <audio_file>")
    main(sys.argv[1])
