'''
This is the class for Text-to-Speech which uses can use two ttsModel (gTTS, googleAPI) to speak
'''
from gtts import gTTS
import os
import playsound
from google.oauth2 import service_account
from google.cloud import texttospeech
from ament_index_python.packages import get_package_share_directory

class TTS():
    def __init__(self, ttsModel="gTTS", api_credential_fileName="ttsCred.json"):
        self.ttsModel = ttsModel
        
        if self.ttsModel == "googleAPI":
            self.api_credential_location = os.path.join( get_package_share_directory('convros_bot'), 'config', api_credential_fileName)
            
            if self.api_credential_location == "": raise Exception("You did not provide google API Credential json file location")
            
            self.credentials = service_account.Credentials.from_service_account_file(self.api_credential_location)
            self.client = texttospeech.TextToSpeechClient(credentials=self.credentials)
            # self.get_logger().warn(f'Using Google Text-to-Speech API')
        elif self.ttsModel == "gTTS":
            # self.get_logger().warn(f'Using gTTS for speech to text service')
            pass


    def speak(self, text):
        if text:
            # self.get_logger().info(f'Speaking: " {text}')
            if self.ttsModel == "gTTS":
                self.synthesize_text_gTTS(text)
            elif self.ttsModel == "googleAPI":
                self.synthesize_text_googleAPI(text)
    
    def synthesize_text_gTTS(self, text):
        # Use gTTS to generate speech and save it to an MP3 file
        tts = gTTS(text=text, lang='en', tld='us', slow=False)
        try:
            tts.save("output.wav")

            # Play the MP3 file
            playsound.playsound("output.wav", True)

            # Remove the MP3 file after playback
            os.remove("output.wav")
        except Exception as e:
            self.get_logger().warn(f'Some Error Occurred!')

    def synthesize_text_googleAPI(self, text):
        """Synthesizes speech from the input string of text using google speech to text api."""
        

        input_text = texttospeech.SynthesisInput(text=text)

        # Note: the voice can also be specified by name.
        # Names of voices can be retrieved with client.list_voices().
        voice = texttospeech.VoiceSelectionParams(
            language_code="en-US",
            name="en-US-Journey-F",
            ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
        )

        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3
        )

        response = self.client.synthesize_speech(
            request={"input": input_text, "voice": voice, "audio_config": audio_config}
        )
        try:
            # The response's audio_content is binary.
            with open("output.mp3", "wb") as out:
                out.write(response.audio_content)
            
            playsound.playsound("output.wav", True)
            # Remove the MP3 file after playback
            os.remove("output.wav")
        except Exception as e:
            self.get_logger().warn(e)

            


def main():
    tts = TTS("gTTS")

    tts.speak("Hi, I am your speaking agent")

if __name__ == '__main__':
    main()
