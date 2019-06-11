# IoT Submodel Source Code

## Installation

1. Make sure you have Ruby and Bundler installed, you can install Ruby  [here](https://www.ruby-lang.org/en/documentation/installation/) and install Bundler [here](http://bundler.io/).
    Note: Please use updated Ruby. In Linux :  `sudo apt-get install ruby-full`.
    
2. Run sinatra install: `sudo apt-get install ruby-sinatra`.

3. Run bundler: `bundle install`.

4. Start the server: `ruby server.rb`.

5. Open http://localhost:4567 in your web browser.

6. Click on the "Connect with SmartThings" link. Enter your SmartThings username and password if not already logged in.

7. Select a Location from the dropdown(UNH Lab in our case), select one or more devices, and click Authorize.

8. The server will simply redirect to a page showing the result of the web request to the SmartApp endpoint.
