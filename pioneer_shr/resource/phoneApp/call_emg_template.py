#!/usr/bin python
# Download the helper library from https://www.twilio.com/docs/python/install
from twilio.rest import Client
import sys


# Your Account Sid and Auth Token from twilio.com/console
# DANGER! This is insecure. See http://twil.io/secure
account_sid = 'put twilio account sid here'
auth_token = 'put twilio auth token here, do not share it on github'
client = Client(account_sid, auth_token)

call = client.calls.create(
                        # url='http://demo.twilio.com/docs/voice.xml',
                        url='http://cs.unh.edu/online server that upload the message files/'+sys.argv[1],
                        to='phone number of emergency person',
                        from_='phone number of twilio account'
                    )

print(call.sid)

