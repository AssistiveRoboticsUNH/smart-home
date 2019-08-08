import mysql.connector
import pandas as pd
#import plotly.graph_objects as go
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import dates as mdates
from pandas.plotting import register_matplotlib_converters
from datetime import datetime


# fig = go.Figure(data=go.Bar(y=[2,1,3]))
# fig.show()

# dev_x = [12,13,14,15,16,17,18,19,20]
#
# dev_y = [50,51,53,54,66,78,90,97,98]
#
# plt.plot(dev_x,dev_y)
#
# plt.show()

# mysql database connection
cnx = mysql.connector.connect(user='root', password='password', host='localhost',
                                database='sensordata')
cursor = cnx.cursor()

querysensorstatusrows = cursor.execute('SELECT time, motionsensor1, motionsensor2, doorsensor, takepill FROM sensorstatus')
querysensorstatusrowsresults = cursor.fetchall()

# tuple to list
listdata = list(querysensorstatusrowsresults)

# list to python dataframe
listdatatoframe = pd.DataFrame(listdata, columns = ['time','motionsensor1','motionsensor2','doorsensor','takepill'])
print(listdatatoframe)
#dataframe to csv file
listdatatoframe.to_csv('/home/alex/Desktop/newsmartthingsiotpart/plot/listtodata.csv')
# print('convert to csv success')


readlistfile = pd.read_csv('listtodata.csv')
readlistfile['time'] = pd.to_datetime(readlistfile['time'])

#convert time to python datetime objects
#readlistfile['time'] = datetime.strptime(readlistfile['time'],"%Y/%m/%d %H:%M:%S")
#print(readlistfile)


#first_twelve = readlistfile[0:12]
register_matplotlib_converters()

fig = plt.figure(4)

ax1motionsensor1 = fig.add_subplot(4,1,1)
ax2motionsensor2 = fig.add_subplot(4,1,2)
ax3doorsensor = fig.add_subplot(4,1,3)
ax4takepill = fig.add_subplot(4,1,4)

ax1motionsensor1.plot(readlistfile['time'], readlistfile['motionsensor1'])
ax1motionsensor1.set_ylabel('M1')
plt.xticks(rotation=60)

ax2motionsensor2.plot(readlistfile['time'], readlistfile['motionsensor2'])
ax2motionsensor2.set_ylabel('M2')
plt.xticks(rotation=60)

ax3doorsensor.plot(readlistfile['time'], readlistfile['doorsensor'])
ax3doorsensor.set_ylabel('D')
plt.xticks(rotation=60)

ax4takepill.plot(readlistfile['time'], readlistfile['takepill'])
ax4takepill.set_ylabel('TP')
plt.xticks(rotation=60)

plt.xlabel('date & time')

#plt.plot(readlistfile['time'], readlistfile['motionsensor1'])

#plt.title('listdata test')

#plt.axis([ , ,0,4])
plt.show()

#=================================================================================

# time = []
# motionsensor1 = []
# motionsensor2 = []
# doorsensor = []
# takepill = []
#
# for x in querysensorstatusrowsresults:
#     time.append(x[0])
#     motionsensor1.append(x[1])
#     motionsensor2.append(x[2])
#     doorsensor.append(x[3])
#     takepill.append(x[4])
#
# print(takepill)
# use
#graphArray = []
# for row in listdatatoframe:
#     startingInfo = str(row).replace(')','').replace('(','').replace('u\'','').replace("'","")
#     splitInfo = startingInfo.split(',')
#     graphArrayAppend = splitInfo[0]+','+splitInfo[3]
#     print (graphArrayAppend)
    #graphArray.append(graphArrayAppend)

#print(graphArray)

cursor.close()
cnx.close()
