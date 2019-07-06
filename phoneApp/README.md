# Phone app using Twilio

# Install:
https://www.fullstackpython.com/blog/make-phone-calls-python.html

# Other pre-configuration:
**Set up for virtualenv**  

Add this to .bashr or .zshrc:  
```bash
source ~/phoneapp/bin/activate
```

**Copy call messages to webserver**  

```bash
scp call_msg.xml tg1034@agate.cs.unh.edu:~/public_html/shr
```

# Usage:
**make phone call and play pre-record message:**  
`python call.py`

