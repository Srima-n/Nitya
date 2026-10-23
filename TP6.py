import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel
from deep_translator import GoogleTranslator

TARGET_LANG = 'en'        # language the user wants, e.g. 'hi', 'ta', 'fr', 'es'
MODEL_SIZE = 'base'       # 'tiny' is faster on a Raspberry Pi, 'small' is more accurate
SAMPLE_RATE = 16000       # Whisper expects 16 kHz audio
CHUNK_SECONDS = 5         # length of each recording
SILENCE_RMS = 0.01        # chunks quieter than this are skipped

model = WhisperModel(MODEL_SIZE, device='cpu', compute_type='int8')


def record_chunk():
    """Record one chunk of mono audio from the default microphone."""
    audio = sd.rec(int(CHUNK_SECONDS * SAMPLE_RATE),
                   samplerate=SAMPLE_RATE, channels=1, dtype='float32')
    sd.wait()
    return audio.flatten()


def transcribe(audio):
    """Detect the spoken language and return the transcript."""
    segments, info = model.transcribe(audio, beam_size=5, vad_filter=True)
    text = ' '.join(seg.text.strip() for seg in segments).strip()
    return text, info.language, info.language_probability


def translate(text, src_lang, target_lang):
    """Translate text into the target language (needs internet access)."""
    if not text or src_lang == target_lang:
        return text
    return GoogleTranslator(source=src_lang, target=target_lang).translate(text)


def main():
    print(f"Listening. Output language: {TARGET_LANG}. Press Ctrl+C to stop.")
    try:
        while True:
            audio = record_chunk()

            # skip silence and background noise
            if np.sqrt(np.mean(audio ** 2)) < SILENCE_RMS:
                continue

            text, lang, prob = transcribe(audio)
            if not text:
                continue

            translated = translate(text, lang, TARGET_LANG)
            print(f"\n[{lang}, {prob:.0%}] {text}")
            print(f"[{TARGET_LANG}] {translated}")

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == '__main__':
    main()
