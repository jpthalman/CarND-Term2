import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import chi2


parser = argparse.ArgumentParser()
parser.add_argument(
    '-i',
    '--infile',
    type=str,
    default='output/out.csv',
    help='The file to analyze. Must be a CSV.'
)
args = parser.parse_args()


def main():
    cwd = os.getcwd()
    data = pd.read_csv(os.path.join(cwd, args.infile), header=0)

    ax = plt.subplot(2,2,1)
    ax.set_title('XY Position')
    ax.scatter(data['Px_gt'], data['Py_gt'], color='orange')
    ax.scatter(data['Px_pred'], data['Py_pred'], color='b')
    ax.legend()

    ax = plt.subplot(2,2,2)
    ax.set_title('XY Velocity')
    ax.scatter(data['Vx_gt'], data['Vy_gt'], color='orange')
    ax.scatter(data['Vx_pred'], data['Vy_pred'], color='b')
    ax.legend()

    nis_radar = data.loc[data.SensorType == 'R']['NIS'].as_matrix()
    chi_95p_3df = np.full(nis_radar.shape, chi2.isf(df=3, q=0.05))

    nis_lidar = data.loc[data.SensorType == 'L']['NIS'].as_matrix()
    chi_95p_2df = np.full(nis_radar.shape, chi2.isf(df=3, q=0.05))

    ax = plt.subplot(2,2,3)
    ax.set_title('Radar NIS')
    ax.plot(nis_radar, 'b')
    ax.plot(chi_95p_3df, 'orange')

    ax = plt.subplot(2,2,4)
    ax.set_title('Lidar NIS')
    ax.plot(nis_lidar, 'b')
    ax.plot(chi_95p_2df, 'orange')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
