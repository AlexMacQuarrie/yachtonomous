# External
import csv
import numpy as np
import matplotlib.pyplot as plt


CSV_PATH = 'boat_log.csv'
OUT_PATH = 'csv_plot.pdf'


def parse_csv() -> None:
    x, x_d, u, u_a, t = [], [], [], [], []

    # Read CSV
    with open(CSV_PATH, 'r') as log_file:
        log_reader = csv.reader(log_file)

        # Parse CSV
        for i, row in enumerate(log_reader):
            if i == 0: continue
            row = [float(i) for i in row]
            x.append(row[:6])
            x_d.append(row[6:12])
            u.append(row[12:14])
            u_a.append(row[14:16])
            t.append(row[-1])

    # Plot results
    t   = np.asarray(t)
    x   = np.asarray(x).T
    x_d = np.asarray(x_d).T
    u   = np.asarray(u).T
    u_a = np.asarray(u_a).T

    num_states, num_inputs = 6, 2

    plt.rc('savefig', format='pdf')
    plt.rc('savefig', bbox='tight')
    fig2 = plt.figure(2)
    fig2.set_figheight(10.0)
    ylabels_states = ['x [m]', 'y [m]', '\u03B8 [deg]', 
                      '\u03B3 [deg]', '\u03C6 [deg]', '\u03B7 [deg]']
    ylabels_inputs = ['\u03C3 [deg/s]', '\u03C9 [deg/s]']
    for i in range(num_states):
        ax2 = plt.subplot(int(str(num_states+num_inputs)+'1'+str(i+1)))
        if i == 4:  # Phi
            plt.plot(t, x[i, :]  *180.0/np.pi, 'C0'  , label='Estimated')
            plt.plot(t, u_a[1, :]*180.0/np.pi, 'C1--', label='Input')
            plt.plot(t, x_d[i, :]*180.0/np.pi, 'C2--', label='Desired')
        elif i == 5:  # Eta
            plt.plot(t, x[i, :]  *180.0/np.pi, 'C0'  , label='Estimated')
            plt.plot(t, u_a[0, :]*180.0/np.pi, 'C1--', label='Inpit')
            plt.plot(t, x_d[i, :]*180.0/np.pi, 'C2--', label='Desired')
        elif 'deg' in ylabels_states[i]:
            plt.plot(t, x[i, :]  *180.0/np.pi, 'C0'  , label='Estimated')
            plt.plot(t, x_d[i, :]*180.0/np.pi, 'C2--', label='Desired')
        else:
            plt.plot(t   , x[i, :]  , 'C0'  , label='Estimated')
            plt.plot(t[0], x[i, 0]  , 'C1--', label='Input')  # Trick to get legend right, not real data
            plt.plot(t   , x_d[i, :], 'C2--', label='Desired')
        plt.grid(color='0.95')
        plt.ylabel(ylabels_states[i])
        plt.setp(ax2, xticklabels=[])
        if i == 0:
            plt.legend()
    for i in range(num_inputs):
        ax2 = plt.subplot(int(str(num_states+num_inputs)+'1'+str(num_states+i+1)))
        if 'deg' in ylabels_inputs[i]:
            plt.step(t, u[i, :]*180.0/np.pi , 'C0'  , label='Estimated')
            plt.plot(t, np.zeros(u.shape[1]), 'C2--', label='Desired')
        else:
            plt.step(t, u[i, :]             , 'C0'  , label='Estimated')
            plt.plot(t, np.zeros(u.shape[1]), 'C2--', label='Desired')
        plt.ylabel(ylabels_inputs[i])
        plt.grid(color='0.95')
        if i != num_inputs-1:
            plt.setp(ax2, xticklabels=[])  
    plt.xlabel('t [s]')
    plt.savefig(OUT_PATH)
    plt.show()


if __name__ == '__main__':
    parse_csv()