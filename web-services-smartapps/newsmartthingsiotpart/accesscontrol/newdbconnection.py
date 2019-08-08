#import mysql.connector

# connect to the database
#connection = mysql.connector.connect(user='root', password='password', host='localhost', database='sensordata')

#connection.close()


import mysql.connector
from mysql.connector import errorcode


print('Enter your username:')
username = input()
#print(username)

print('Enter your password:')
password = input()
#print(password)
#Error Code: 1064. You have an error in your SQL syntax; check the manual that corresponds to your MySQL server version for the right syntax to use near 'group) values('chris', md5('chrisabc'), 'abc', 'caregiver')' at line 1
#Error Code: 1064. You have an error in your SQL syntax; check the manual that corresponds to your MySQL server version for the right syntax to use near 'group) values('kevin', 'kevin', 'def', 'caregiver')' at line 1



# connection
cnx = mysql.connector.connect(user='root', password='password', host='localhost',
                                database='sensordata')

cursor = cnx.cursor()
querybyusername = cursor.execute("""SELECT password FROM usersclass WHERE username = %s""", (username,))
querybyusernameresult = cursor.fetchone()
#print(type(querybyusernameresult))
#return a tuple

if querybyusernameresult:
    # user exist
    # print(querybyusernameresult)
    if querybyusernameresult[0] == password:
        # show table
        if username == 'chris':
            querycaregiver = ("SELECT * FROM sensorstatus")
            cursor.execute(querycaregiver)
            caregiverresult = cursor.fetchall()
            for x in caregiverresult:
                print(x)
            print("caregiver login success")
        if username == 'tim':
            querycaregiver = ("SELECT * FROM sensorstatus")
            cursor.execute(querycaregiver)
            caregiverresult = cursor.fetchall()
            for x in caregiverresult:
                print(x)
            print("caregiver login success")
        if username == 'james':
            querycaregiver = ("SELECT * FROM sensorstatus")
            cursor.execute(querycaregiver)
            caregiverresult = cursor.fetchall()
            for x in caregiverresult:
                print(x)
            print("caregiver login success")
        if username == 'kevin':
            queryrelatives = ("select patient_id, patient_name, aes_decrypt(SSN, 'ssn'), aes_decrypt(DOB, 'dob'), aes_decrypt(home_address, 'homeaddress'), ems_contact from patientinformation")
            cursor.execute(queryrelatives)
            relativesresult = cursor.fetchall()
            for x in relativesresult:
                print(x)
            print("relatives login success")
        if username == 'ross':
            queryrelatives = ("select patient_id, patient_name, aes_decrypt(SSN, 'ssn'), aes_decrypt(DOB, 'dob'), aes_decrypt(home_address, 'homeaddress'), ems_contact from patientinformation")
            cursor.execute(queryrelatives)
            relativesresult = cursor.fetchall()
            for x in relativesresult:
                print(x)
            print("relatives login success")
    else:
        # wrong password
        print('Wrong user or password!')
else:
    print('Wrong user or password!')



# query selected table
# query = ("SELECT * FROM sensorstatus")
# cursor.execute(query)
# result = cursor.fetchall()
# for x in result:
#   print(x)

# close cursor and connection
cursor.close()
cnx.close()
