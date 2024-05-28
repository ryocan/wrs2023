from pydub import AudioSegment
from pydub.playback import play

song = AudioSegment.from_wav("棚前作業.wav")
play(song)
