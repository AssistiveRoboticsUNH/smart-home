require 'bundler/setup'
require 'sinatra'
require 'oauth2'
require 'json'
require "net/http"
require "uri"

# Our client ID and secret, used to get the access token
CLIENT_ID = '891e1469-e265-4b81-b3f7-ec0ff12c2626'
CLIENT_SECRET = 'ac4e4ea5-0f73-471b-961a-8cba50cd9467'

puts "0.5 ".inspect

# We'll store the access token in the session
use Rack::Session::Pool, :cookie_only => false

puts "0.6 ".inspect

# This is the URI that will be called with our access
# code after we authenticate with our SmartThings account
redirect_uri = 'http://localhost:4567/oauth/callback'

puts "0.7 ".inspect

# This is the URI we will use to get the endpoints once we've received our token
endpoints_uri = 'https://graph.api.smartthings.com/api/smartapps/endpoints'

puts "0.8 ".inspect

# just store the token globally
# This is a HORRIBLE idea in a real application, of course.
# But, it works for our example
#thetoken = ''
puts "1. just store the token globally".inspect

options = {
  site: 'https://graph.api.smartthings.com',
  authorize_url: '/oauth/authorize',
  token_url: '/oauth/token'
}

puts "1.5 ".inspect

# use the OAuth2 module to handle OAuth flow
client = OAuth2::Client.new(CLIENT_ID, CLIENT_SECRET, options)

puts "2. use the OAuth2 module to handle OAuth flow".inspect

def authenticated?
    
    puts "2.4 ".inspect

  session[:access_token]
end

puts "2.5 use the OAuth2 module to handle OAuth flow".inspect


# handle requests to the application root
get '/' do
    
    puts "2.6 ".inspect
    
  %(<a href="/authorize">Connect with SmartThings</a>)
end

puts "3. handle requests to the application root".inspect

# handle requests to /authorize URL
get '/authorize' do
  # Use the OAuth2 module to get the authorize URL.
  # After we authenticate with SmartThings, we will be redirected to the
  # redirect_uri, including our access code used to get the token
  
  puts "3.5 ".inspect

  url = client.auth_code.authorize_url(redirect_uri: redirect_uri, scope: 'app')
  
  puts "3.6 " + url.inspect
  #stop here to provide samsung account and password & select certain devices
  redirect url
  
  puts "3.7 ".inspect

end

puts "4. Use the OAuth2 module to get the authorize URL. After we authenticate with SmartThings, we will be redirected to the redirect_uri, including our access code used to get the token".inspect

# hanlde requests to /oauth/callback URL. We
# will tell SmartThings to call this URL with our
# authorization code once we've authenticated.
get '/oauth/callback' do
  # The callback is called with a "code" URL parameter
  # This is the code we can use to get our access token
  
  puts "4.5 ".inspect

  code = params[:code]

  puts "5. hanlde requests to /oauth/callback URL. We will tell SmartThings to call this URL with our authorization code once we've authenticated.".inspect
  
  puts 'headers: ' + headers.to_hash.inspect

  puts "5.5 ".inspect
  
  # Use the code to get the token.
  response = client.auth_code.get_token(code, redirect_uri: redirect_uri, scope: 'app')

  puts "5.6 ".inspect

  # now that we have the access token, we will store it in the session
  session[:access_token] = response.token

  puts "5.7 ".inspect

  # debug - inspect the running console for the
  # expires in (seconds from now), and the expires at (in epoch time)
  puts 'TOKEN EXPIRES IN ' + response.expires_in.to_s
  puts 'TOKEN EXPIRES AT ' + response.expires_at.to_s
  
  puts "5.8 ".inspect

  redirect '/getcontacts'
  
  puts "5.9 ".inspect

end

puts "6. debug - inspect the running console for the expires in (seconds from now), and the expires at (in epoch time)".inspect

# handle requests to the /getcontacts URL. This is where
# we will make requests to get information about the configured
# contacts.
get '/getcontacts' do
  # If we get to this URL without having gotten the access token
  # redirect back to root to go through authorization
  
  puts "6.1 ".inspect

  if !authenticated?
    
    puts "6.2 ".inspect

    redirect '/'
    
    puts "6.3 ".inspect

  end

  puts "7. handle requests to the /getcontacts URL. This is where we will make requests to get information about the configured contacts.".inspect
  
  token = session[:access_token]

  # make a request to the SmartThins endpoint URI, using the token,
  # to get our endpoints
  url = URI.parse(endpoints_uri)
  req = Net::HTTP::Get.new(url.request_uri)

  puts "8. make a request to the SmartThins endpoint URI, using the token, to get our endpoints".inspect
  
  # we set a HTTP header of "Authorization: Bearer <API Token>"
  req['Authorization'] = 'Bearer ' + token

  http = Net::HTTP.new(url.host, url.port)
  http.use_ssl = (url.scheme == "https")

  response = http.request(req)
  json = JSON.parse(response.body)

  # debug statement
  puts json

  # get the endpoint from the JSON:
  uri = json[0]['uri']

  # now we can build a URL to our WebServices SmartApp
  # we will make a GET request to get information about the contacts
  contactsUrl = uri + '/contacts'

  # debug
  puts "CONTACTS ENDPOINT: " + contactsUrl

  getcontactsURL = URI.parse(contactsUrl)
  getcontactsReq = Net::HTTP::Get.new(getcontactsURL.request_uri)
  getcontactsReq['Authorization'] = 'Bearer ' + token

  getcontactsHttp = Net::HTTP.new(getcontactsURL.host, getcontactsURL.port)
  getcontactsHttp.use_ssl = true

  contactsStatus = getcontactsHttp.request(getcontactsReq)

  '<h3>Response Code</h3>' + contactsStatus.code + '<br/><h3>Response Headers</h3>' + contactsStatus.to_hash.inspect + '<br/><h3>Response Body</h3>' + contactsStatus.body
  
 
end

puts "9.0 ".inspect

