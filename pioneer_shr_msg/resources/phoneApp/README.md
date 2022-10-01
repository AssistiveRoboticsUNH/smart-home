# Phone app using Twilio

# Install:
https://www.fullstackpython.com/blog/make-phone-calls-python.html

# Other pre-configuration:
**Set up virtualenv for resolve all dependency of twilio library**  

Add this to .bashr or .zshrc:  
```bash
source ~/phoneapp/bin/activate
```
(not sure if we really need this. It seems work even without this step)

# Usage:
**edit or create pre-record message:**  
`vim call_msg.xml`

**Copy call messages to webserver**  
```bash
./upload_messages.sh
```

**make phone call and play pre-record message:**  
`python call.py call_msg.xml`

