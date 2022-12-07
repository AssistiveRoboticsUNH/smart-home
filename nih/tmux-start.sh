tmux new-session \; \
  send-keys 'smarthome' C-m \; \
  split-window -v \; \
  send-keys 'smarthome' C-m \; \
  split-window -h \; \
  send-keys 'smarthome' C-m \; \
  select-pane -t 0 \; \
  split-window -h \; \
  send-keys 'smarthome' C-m \; 

  
#“-t” specifies the target window, which in this case is simply a session name so that Tmux may use the subsequent unused index.
#The “=” ensures an exact match.
#The “-n” option names the window
#The “-c” option specifies the directory.

#Does that. Basicaly knowing your way around with split-window and select-pane is all you need. 
#It's also handy to pass with -p 75 a percentage size of the pane created by split-window to have more control over the size of the panes.

