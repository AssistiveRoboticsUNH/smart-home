'''
To List out the available audio input device and their device_id run this script
'''
import speech_recognition as sr

for index, name in enumerate(sr.Microphone.list_microphone_names()):
    print(f"Microphone with index {index}: {name}")