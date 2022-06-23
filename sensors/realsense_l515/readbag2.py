import rosbag
# bag = rosbag.Bag('outputs\\bag\\20211007_141948.bag')

with rosbag.Bag('outputs\\bag\\20211007_130643.bag', 'r') as bag:

    for (topic, msg, t) in bag.read_messages():
        print(topic)
        print("--")

