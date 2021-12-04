import pandas as pd
import numpy as np

def odom_csv2np():
    data = pd.read_csv("./data/odom.csv")
    pos = np.vstack((
        data["field.pose.pose.position.x"],
        data["field.pose.pose.position.y"],
        data["field.pose.pose.position.z"]
    )).T
    att = np.vstack((
        data["field.pose.pose.orientation.x"],
        data["field.pose.pose.orientation.y"],
        data["field.pose.pose.orientation.z"],
        data["field.pose.pose.orientation.w"]
    )).T

    print(pos[0:5])

if __name__ == "__main__":
    odom_csv2np()