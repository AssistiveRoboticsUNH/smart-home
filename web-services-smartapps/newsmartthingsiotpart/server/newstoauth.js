/*
 OAUTH Client
 needs
 npm install express
 npm install request
 npm install JSON
 npm install mysql
 npm install sillytime
 npm install node-schedule
 npm install child_process
*/

var CLIENT_ID = "891e1469-e265-4b81-b3f7-ec0ff12c2626";
var CLIENT_SECRET = "ac4e4ea5-0f73-471b-961a-8cba50cd9467";
var TOKEN = null;
var ENDPOINT = null;

var process = require('child_process');
var schedule = require('node-schedule');
var sillytime = require('silly-datetime');
var mysql = require('mysql');
var request = require('request');
var express = require('express'),
  app = express();


var endpoints_uri = 'https://graph.api.smartthings.com/api/smartapps/endpoints';
const credentials = {
  client: {
    id: CLIENT_ID,
    secret: CLIENT_SECRET
  },
  auth: {
    tokenHost: 'https://graph.api.smartthings.com'
  }
};

var oauth2 = require('simple-oauth2').create(credentials)
// Authorization uri definition
var authorization_uri = oauth2.authorizationCode.authorizeURL({
  redirect_uri: 'http://localhost:4567/oauth/callback',
  scope: 'app',
  state: '3(#0/!~'
});

// Initial page redirecting to authorizaiton page
app.get('/auth', function (req, res) {
  res.redirect(authorization_uri);
});

app.get('/getcontacts', function(req, res) {
  var currenttime = sillytime.format(new Date(), 'YYYY-MM-DD HH:mm:ss');
  //console.log(currenttime);
  //res.send('<p>abc</p>');
  var options = { method: 'GET',
    url: ENDPOINT + '/contacts',
    headers:
     { 'cache-control': 'no-cache',
       Connection: 'keep-alive',
       'accept-encoding': 'gzip, deflate',
       cookie: 'JSESSIONID=7C4BEE912E029049C81BC8A39E76FAAF-n2',
       Host: 'graph-na04-useast2.api.smartthings.com:443',
       'Postman-Token': 'fcb8cb60-cf8c-496d-83d2-1d69f0da55b1,57a8994a-f97d-4335-9181-47c72f3447c1',
       'Cache-Control': 'no-cache',
       Accept: '*/*',
       'User-Agent': 'PostmanRuntime/7.13.0',
       Authorization: 'Bearer ' + TOKEN } };

  request(options, function (error, response, body) {
    if (error) throw new Error(error);

    //console.log(body);
    console.log();
    var sensorjson = JSON.parse(body);
    var doorsensorname = sensorjson[0].name;
    var doorsensorstatus = sensorjson[0].value;
    var motionsensor1name = sensorjson[1].name;
    var motionsensor1status = sensorjson[1].value;
    var motionsensor2name = sensorjson[2].name;
    var motionsensor2status = sensorjson[2].value;
    // console.log(sensorjson);
    // console.log(doorsensorname);
    // console.log(doorsensorstatus);
    // console.log(motionsensor1name);
    // console.log(motionsensor1status);
    // console.log(motionsensor2name);
    // console.log(motionsensor2status);
    //change value type to 0 or 1;
    if (doorsensorstatus == 'open') {
      doorsensorstatus = '1';
    }else {
      doorsensorstatus = '0';
    }
    if (motionsensor1status == 'active') {
      motionsensor1status = '1';
    }else {
      motionsensor1status = '0';
    }
    if (motionsensor2status == 'active') {
      motionsensor2status = '1';
    }else {
      motionsensor2status = '0';
    }

    //takepill column generate condition
    var takepillstatus;
    if (motionsensor1status == '1') {
      takepillstatus = '1';
    } else {
      takepillstatus = '0';
    }
    //console.log(takepillstatus);

    //change to boolean value


    //mysql config
    //SELECT * FROM sensorstatus;
    var connection = mysql.createConnection({
        host : 'localhost',
        user : 'root',
        password : 'password',
        database : 'sensordata'
    });
    connection.connect(function(err){
        if(err) throw err;
        console.log('connect success');
    });

    //insert sensorstatus table statement
    var sqlinsert = 'INSERT INTO sensorstatus(time, motionsensor1, motionsensor2, doorsensor, takepill) VALUES(' + "'" + currenttime + "'" + ',' + "'" + motionsensor1status + "'" + ',' + "'" + motionsensor2status + "'" + ',' + "'" + doorsensorstatus + "'" + ',' + "'" + takepillstatus + "'" + ')';
    //var sqlselectsensorstatus = 'SELECT * FROM sensorstatus';
    //var sqlselectsensorstatusresults = connection.query(sqlselectsensorstatus);
    //console.log(sqlselectsensorstatusresults);
    console.log(currenttime);
    connection.query(sqlinsert);
    connection.end(function(err){
        if(err) throw err;
        //console.log('connect end');
    });
    res.send(body);
  });
});

//automatic call getcontacts per minute
function callgetcontacts(){
  schedule.scheduleJob('* * * * * *', function(){
    var options = { method: 'GET',
    url: 'http://localhost:4567/getcontacts',
    headers:
     { 'cache-control': 'no-cache',
       Connection: 'keep-alive',
       'accept-encoding': 'gzip, deflate',
       Host: 'localhost:4567',
       'Postman-Token': 'e96810d4-11d1-461b-8328-eae30986a75b,892029e1-471d-4836-9acc-61141c772341',
       'Cache-Control': 'no-cache',
       Accept: '*/*',
       'User-Agent': 'PostmanRuntime/7.15.0' } };

  request(options, function (error, response, body) {
    if (error) throw new Error(error);

    console.log('callgetcontacts success');
  });
  });
}
callgetcontacts();


//automatic database backup function
var backuptime = sillytime.format(new Date(), 'YYYY-MM-DD HH:mm:ss');

function sqlautobackup(){
  schedule.scheduleJob('30 * * * * *', function(){
    process.exec('./mysqlbackup.sh', function(error, stdout, stderr){
      if(error !== null) {
        console.log('exec error: ' + error);
      }
    });
    console.log('database automaticdc everyday backup successfully ');
  });
}
sqlautobackup();

// Callback service parsing the authorization token and asking for the access token
app.get('/oauth/callback', async function (req, res) {
  var code = req.query.code;
  // console.log('/callback got code' + code);
  console.log(code)
  const tokenConfig = {
    code: code,
    redirect_uri: 'http://localhost:4567/oauth/callback'
  };
  // get token
  try {
    const result = await oauth2.authorizationCode.getToken(tokenConfig)
    const accessToken = oauth2.accessToken.create(result);
    // save token to TOKEN global variable
    TOKEN = accessToken.token.access_token;

    console.log(TOKEN)
    //res.redirect('/getcontacts')
  } catch (error) {
    console.log('Access Token Error', error.message);
  }

  var options = { method: 'GET',
  url: 'https://graph.api.smartthings.com/api/smartapps/endpoints',
  headers:
   { 'cache-control': 'no-cache',
     Connection: 'keep-alive',
     'accept-encoding': 'gzip, deflate',
     cookie: 'JSESSIONID=1022B9A67D1B572F9B0F4998B47B43A2-n2',
     Host: 'graph.api.smartthings.com',
     'Postman-Token': 'ab9e4694-6b38-4809-912b-da5aa3769fbc,8222662b-5ac4-49e6-89c0-b23487bed8ee',
     'Cache-Control': 'no-cache',
     Accept: '*/*',
     'User-Agent': 'PostmanRuntime/7.13.0',
     Authorization: 'Bearer ' + TOKEN } };

  request(options, function (error, response, body) {
    if (error) throw new Error(error);

    var bodyjson = JSON.parse(body);
    ENDPOINT = bodyjson[0].uri;
    res.redirect('/getcontacts');
    //console.log(ENDPOINT);
    //console.log(body);
  });
});

app.get('/getdatabase', function(req,res) {
  //query from database
  //html template list
  //log first
});

app.get('/', function (req, res) {
  res.send('<a href="/auth">Connect with SmartThings</a>');
});

app.listen(4567);
console.log('Express server started on port 4567');
