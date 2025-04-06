import discord
import asyncio
import threading


class DiscordClient(discord.Client):
    def __init__(self, ready_event, target_channel_id, command_callback=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ready_event = ready_event
        self.target_channel_id = target_channel_id
        self.command_callback = command_callback

    async def on_ready(self):
        print(f"Discord Bot logged in as {self.user}")
        self.ready_event.set()

    async def on_message(self, message):
        # print(f"Received message: '{message.content}' Author {message.author}")
        # Ignore messages sent by the bot itself.
        if message.author == self.user:
            return

        # Only listen to messages in the target channel.
        if message.channel.id == self.target_channel_id:
            content = message.content.strip()
            # Call the external callback.
            callback = self.command_callback
            if callback != None:
                # If the callback is a coroutine, await it; otherwise, call it normally.
                if asyncio.iscoroutinefunction(callback):
                    await callback(content)
                else:
                    callback(content)
        

class DiscordNotifier:
    """
    A class to send messages to a Discord channel using a bot.
    
    Usage:
    
        from discord_notifier import DiscordNotifier
        
        TOKEN = "YOUR_DISCORD_BOT_TOKEN"
        CHANNEL_ID = 123456789012345678  # Replace with your channel ID
        
        notifier = DiscordNotifier(token=TOKEN, channel_id=CHANNEL_ID)
        notifier.send_message("Hello, Discord!")
    """
    def __init__(self, token, channel_id, on_message_callback=None, *args, **kwargs):
        self.token = token
        self.channel_id = channel_id
        self.ready_event = threading.Event()
        
        # Setup Discord client with default intents
        intents = discord.Intents.default()
        intents.messages = True
        intents.message_content = True
        self.client = DiscordClient(ready_event=self.ready_event,
                                    target_channel_id=self.channel_id,
                                    command_callback=on_message_callback,
                                    intents=intents)
        
        # Create a new event loop in a separate thread for Discord
        self.loop = None
        self.thread = threading.Thread(target=self._run_client, daemon=True)
        self.thread.start()
        
        # Wait until the client is ready (with a timeout, if needed)
        if not self.ready_event.wait(timeout=10):
            print("Warning: Discord client did not become ready in time.")

    def _run_client(self):
        """Internal method to run the Discord client in a new event loop."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.client.start(self.token))
        except Exception as e:
            print("Discord client encountered an error:", e)

    async def send_message(self, message):
        """
        Sends a message to the specified Discord channel.
        
        Args:
            message (str): The message to be sent.
        """
        async def _send():
            # Try to get the channel; if not found, fetch it
            channel = self.client.get_channel(self.channel_id)
            if channel is None:
                try:
                    channel = await self.client.fetch_channel(self.channel_id)
                except Exception as e:
                    print(f"Error fetching channel: {e}")
                    return
            await channel.send(message)

        if self.loop is not None:
            # self.loop.create_task(_send())
            future = asyncio.run_coroutine_threadsafe(_send(), self.loop)
            try:
                future.result(timeout=2)
            except Exception as e:
                print(f"Failed to send message: {e}")
        else:
            print("Discord event loop is not running.")

    def close(self):
        """
        Closes the Discord client connection.
        """
        if self.loop is not None:
            # asyncio.run_coroutine_threadsafe(self.client.close(), self.loop)
            asyncio.run_coroutine_threadsafe(self.client.close(), self.loop).result()
            self.loop.call_soon_threadsafe(self.loop.stop)

