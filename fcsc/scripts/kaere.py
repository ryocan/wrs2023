from pydub import AudioSegment
from pydub.playback import play

song = AudioSegment.from_wav("待ってください.wav")
play(song)
