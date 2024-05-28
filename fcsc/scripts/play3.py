from pydub import AudioSegment
from pydub.playback import play

song = AudioSegment.from_wav("回避.wav")
play(song)
