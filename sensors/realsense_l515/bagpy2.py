from bagpy import bagreader


b = bagreader('outputs\\bag\\20211007_135307.bag')
# b = bagreader('outputs\\bag\\OS1-64_city1.bag')

print(b.reader)
# print(b.topic_table)
data = b.message_by_topic('/device_0/sensor_0/l500_data')

print(data)
# csvfiles = []
# for t in b.topics:
#     print(t)
#     # data = b.message_by_topic(t)
#     # csvfiles.append(data)

# b.message_types('/device_0/sensor_1/option/White_Balance/value')
# print("File saved: {}".format(data))


# https://rahulbhadani.medium.com/reading-ros-messages-from-a-bagfile-in-python-b006538bb520