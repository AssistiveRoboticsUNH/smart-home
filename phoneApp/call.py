# Download the helper library from https://www.twilio.com/docs/python/install
from twilio.rest import Client


# Your Account Sid and Auth Token from twilio.com/console
# DANGER! This is insecure. See http://twil.io/secure
account_sid = 'ACdac3f305e5f9ae571d99f9f3acdb91b5'
auth_token = '8187d159382b0a2719caa9aa0b20aa1e'
client = Client(account_sid, auth_token)

call = client.calls.create(
                        url='http://demo.twilio.com/docs/voice.xml',
                        to='+16262026180',
                        from_='+15624541623'
                    )

print(call.sid)

