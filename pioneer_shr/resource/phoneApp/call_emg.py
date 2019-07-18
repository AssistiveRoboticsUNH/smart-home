#!/usr/bin python
# Download the helper library from https://www.twilio.com/docs/python/install
from twilio.rest import Client
import sys


# Your Account Sid and Auth Token from twilio.com/console
# DANGER! This is insecure. See http://twil.io/secure
account_sid = 'AC91631fae45e5156720b48e5da8bc84ef'
auth_token = '1061aca46ea400429a9b5287be525ace'
client = Client(account_sid, auth_token)

call = client.calls.create(
                        # url='http://demo.twilio.com/docs/voice.xml',
                        url='http://cs.unh.edu/~tg1034/shr/'+sys.argv[1],
                        to='+16262026180',
                        from_='+15624541623'
                    )

print(call.sid)

