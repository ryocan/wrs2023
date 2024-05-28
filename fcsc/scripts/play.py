from pydub import AudioSegment
from pydub.playback import play

song = AudioSegment.from_wav("be_careful_fcsc.wav")
play(song)
