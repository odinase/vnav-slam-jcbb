import numpy as np
from scipy.stats import chi2
import matplotlib.pyplot as plt
import sys

def association_statistics():
    assert len(sys.argv) == 2 and sys.argv[1] in ["jcbb", "ml", "gt"], f"Must have extra argument for what method to check, use 'jcbb', 'ml' or 'gt', arguments are now {sys.argv}"

    method = sys.argv[1]

    assos = dict()
    with open('./logs/'+method+'/april_lmk_assos.txt') as f:
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

    nis_consistency = np.loadtxt('./logs/'+method+'/nis_consistency.txt')
    nis, dof = nis_consistency.T
    alpha = 0.05 # Hypothesis level

    dof = dof.astype(int)
    valid_hypotheses = dof != 0
    dof = dof[valid_hypotheses]
    nis = nis[valid_hypotheses]

    bounds = np.array([alpha/2, 1-alpha/2])

    lower, upper = chi2.cdf(bounds.reshape(-1, 1), dof)

    fig, ax = plt.subplots()
    ax.semilogy(lower, '--', label='lower')
    ax.semilogy(upper, '--', label='upper')
    ax.semilogy(nis, label='hypothesis nis')
    num_inside = ((lower <= nis) & (nis <= upper)).sum()
    ax.set_title(f"NIS inside {(1-alpha)*100:.2f}% confidence interval: {num_inside/nis.shape[0]*100:.3f}%")

    plt.show()

    return assos

if __name__ == "__main__":
    association_statistics()