import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from shr_msgs.action import DeepFakeRequest

import os
from elevenlabslib.helpers import *
from elevenlabslib import *
import playsound
import tempfile


class DeepFakeActionServer(Node):

    def __init__(self):
        super().__init__('deep_fake_action_server')
        self._action_server = ActionServer(
            self,
            DeepFakeRequest,
            'read_script',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        self.get_logger().info('Playing audio for ' + request.voice_name + " reading " + request.script_name)
        # print('here1')
        success = self.playScript(request.script_name, request.voice_name)
        # print('here1=2')
        result = DeepFakeRequest.Result()

        if success:
            result.status = "success"
            goal_handle.succeed()
        else:
            result.status = 'failure'
            goal_handle.abort()

        return result

    # plays audio of script
    # scriptName: string, name of script
    # voiceName: string, name of voice, must match with directory name
    # apikey: elevenlabs api profile key
    # returns True if script played successfuly, False if an error occured trying to play
    def playScript(self, scriptName, voiceName):
        # login user
        # print('here13')
        apikey = os.getenv("APIKEY")
        user = ElevenLabsUser(apikey)
        # print('here133')
        # store file path for reaching folder containing generated audio for voiceName
        basePath = os.path.dirname(os.path.realpath(__file__))
        voicePath = basePath + f'/voiceProfiles/{voiceName}'
        filePath = voicePath + f'/downloadedPresets'
        # store path for location of script text file
        scriptPath = basePath + f'/scripts/{scriptName}.txt'
        print('dddd', scriptPath)

        # check to make sure voice profile exists
        if os.path.exists(voicePath):
            try:
                print('path for audio file')

                tempPath = filePath + f"/{scriptName}.wav"
                print(tempPath)

                # wavfilename = self.create_wav_from_text(tempPath)
                # os.system('vlc ' + wavfilename + ' vlc://quit')
                play_audio_bytes(open(tempPath, "rb").read(), False)
                # playsound('temPath')
                os.system('vlc ' + tempPath + ' vlc://quit')

            except FileNotFoundError:
                print(f"No file found for script {scriptName} at filepath {tempPath}, generating audio and saving")
                voiceProfile = user.get_voices_by_name(voiceName)[0]  # grab user from elevenlabs

                # read script file 
                with open(scriptPath, 'r') as file:
                    script = file.read().replace('\n', ' ')

                # Generate the audio using the script
                audioData = voiceProfile.generate_and_play_audio(script, stability=0.4, playInBackground=False)

                # Save audio
                savePath = filePath + f"/{scriptName}.wav"
                # save_audio_bytes(audioData, savePath, outputFormat="wav")
                with open(savePath, mode='bx') as file:
                    file.write(audioData)


        # no profile, abort
        else:
            print(f"No profile found for {voiceName}.")
            return False

        return True

    def create_wav_from_text(self, file_path):
        (wavfile, wavfilename) = tempfile.mkstemp(
            prefix='sound_play', suffix='.wav')

        # Create a gTTS object with the text and language
        # Path to the file containing the text you want to convert
        # Open the text file and read its contents
        with open(file_path, 'r') as f:
            mytext = f.read()
        tts_obj = gTTS(text=mytext, lang='en', slow=False)

        # Save the generated speech as a WAV file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            wavfilename = f.name
            tts_obj.save(wavfilename)
        # os.system("pico2wave -l en-US -w" + wavfilename + f' "{data.arg}"')
        wavfilename_new = wavfilename.replace('.wav', '')
        wavfilename_new += '_new.wav'
        os.system("ffmpeg -i " + wavfilename + " -af areverse,apad=pad_dur=500ms,areverse " + wavfilename_new)
        wavfilename = wavfilename_new
        return wavfilename


def main(args=None):
    rclpy.init(args=args)

    action_server = DeepFakeActionServer()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
