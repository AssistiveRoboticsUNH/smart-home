#!/bin/bash
# get current time
date_now=$(date "+%Y%m%d-%H%M%S")
backUpFolder=/home/alex/dumps
username="root"
password="password"
db_name="sensordata"
# define backup file name
fileName="${db_name}_${date_now}.sql"
# define backup file directory
backUpFileName="${backUpFolder}/${fileName}"
echo "starting backup mysql ${db_name} at ${date_now}."
mysqldump -u${username} -p${password}  --lock-all-tables --flush-logs ${db_name} > ${backUpFileName}
# enter backUpFolder directory
cd ${backUpFolder}
# zip backup file
tar zcvf ${fileName}.tar.gz ${fileName}

# use nodejs to upload backup file other place
#NODE_ENV=$backUpFolder@$backUpFileName node /home/tasks/upload.js
date_end=$(date "+%Y%m%d-%H%M%S")
echo "finish backup mysql database ${db_name} at ${date_end}."
