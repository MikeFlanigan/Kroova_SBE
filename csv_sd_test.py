import csv 

writer = csv.writer(open("/path/to/my/csv/file", 'w'))
for row in data:
    if counter[row[0]] >= 4:
        writer.writerow(row)