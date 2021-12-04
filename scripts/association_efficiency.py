import numpy as np

def association_statistics():
    assos = dict()
    with open('./logs/april_lmk_assos.txt') as f:
        for line in f:
            s = line.split(' ')[:-1] # Last element is always unwanted whitespace
            apriltag_id = int(s[0])
            landmarks = [int(ss) for ss in s[1:]]
            assos[apriltag_id] = landmarks

    for apriltag_id, lmks in assos.items():
        print(f"apriltag_id: {apriltag_id}\n\tlmks: {lmks}")

    num_apriltags = len(assos)
    failed_assos = len([
        l for l in assos.values() if len(l) > 1
    ])
    num_landmarks = sum([
        len(l) for l in assos.values()
    ])
    print(f"Number of apriltags in dataset: {num_apriltags}")
    print(f"Number of apriltags with multiple landmarks associated with it (failed associations): {failed_assos}")
    print(f"{failed_assos/num_apriltags*100.0:.2f}% of the apriltags")
    print(f"Number of landmarks: {num_landmarks}")

    return assos

if __name__ == "__main__":
    association_statistics()