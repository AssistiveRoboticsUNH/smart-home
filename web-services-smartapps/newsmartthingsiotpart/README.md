# new smartthings iot part

## prerequest 

### install mysql database
https://support.rackspace.com/how-to/installing-mysql-server-on-ubuntu/

## Install:


## 1. nodejs module install 

### add nodejs PPA
sudo apt-get install curl python-software-properties
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -

### check version
node -v
npm -v


### nodejs module
sudo apt-get install nodejs

### express module 
sudo npm install express

### request module
sudo npm install express

### JSON module 
sudo npm install JSON

### mysql module 
sudo npm mysql

### sillytime module
sudo npm install silly-datetime

### node-schedule module
sudo npm install node-schedule

### child_process module
sudo npm install child_process

## 2. Access Control Module install

### pip install
sudo apt install python-pip 
sudo apt install python3-pip

### mysql-connector install
sudo pip3 install mysql-connector-python


## 3. Matpoltlib module install 

### pandas install 
sudo pip3 install pandas

### matplotlib install
sudo apt-get install python3-matplotlib


## Instruction 

### 1. Start the server
        run ~newsmartthingsiotpart$ cd server/
        run ~newsmartthingsiotpart/server$ node newstoath.js

### 2. Oauthenticate
        open a browser and go to the 'localhost:4567'
        click 'connectwithsmartthings' button
        enter the username and password of your samsung account
        select the devices in drop-down list
        click 'authorize' button
        
### 3. plot sensors' status
        run ~newsmartthingsiotpart$ cd plot/
        run ~newsmartthingsiotpart/plot$ python3 matplotlibshow.py

### 4. check database using accesscontrol
        run ~newsmartthingsiotpart$ cd accesscontrol/
        run ~newsmartthingsiotpart/accesscontrol$ python3 newdbconnection.py
        type in username 'enter' and type in password 'enter'
        
### 5. check database backup 
        run ~$ cd dumps/

### 6. database operate instruction
	~$ mysql -u root -p

	show databases;
	use sensordata;
	show tables;
	select * from users;
	select * from patientinformation;
	select * from sensorstatus;

	
        
        
        
        
        
        
            
