'''
This is the class of RealtimeSTT which uses Whisper model to detect real-time speech.
'''

from RealtimeSTT import AudioToTextRecorder



class speechRecognizer():
    def __init__(self):
        # print("Wait until it says 'speak now'")
        print("Intializing speech recognizer")
        self.recorder = AudioToTextRecorder(model = 'base.en', input_device_index=5, spinner=True, 
                                       min_gap_between_recordings=0.5, silero_sensitivity=0.8, webrtc_sensitivity=2,
                                       min_length_of_recording = 0.5)
    def process_text(self, text):
        print(text)
    
    def stop(self):
        self.recorder.stop()
        self.recorder.interrupt_stop_event.set()

def main():
    recognizer =  speechRecognizer()

    while True:
        recognizer.recorder.text(recognizer.process_text)


if __name__ == '__main__':
    main()

# silero_sensitivity (float, default=0.6): Sensitivity for Silero's voice activity detection ranging from 0 (least sensitive) to 1 (most sensitive). Default is 0.6.
# webrtc_sensitivity (int, default=3): Sensitivity for the WebRTC Voice Activity Detection engine ranging from 0 (least aggressive / most sensitive) to 3 (most aggressive, least sensitive). Default is 3.
# min_length_of_recording (float, default=1.0): Specifies the minimum duration in seconds that a recording session should last to ensure meaningful audio capture, preventing excessively short or fragmented recordings.