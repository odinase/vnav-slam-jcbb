if __name__ == "__main__":
    files_to_remove_withspace = ["landmarks", "odom", "path"]
    for f_name in files_to_remove_withspace:
        f_new = open("./logs/jcbb/" + f_name + "_good.txt", "w")
        with open("./logs/jcbb/" + f_name + ".txt") as f:
            for k, line in enumerate(f):
                f_new.write(str(k) + " " +  line[:-2] + "\n")
        f_new.close()